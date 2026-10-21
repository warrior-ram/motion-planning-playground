import { AnimatePresence, motion, useReducedMotion } from 'motion/react'
import { useCallback, useMemo, useState } from 'react'
import type { ArmAlgorithm, Obstacle, Point } from '../engine/types'
import { armConfigValid, forwardKinematics, inverseKinematics, planArm, type TimedArmPlan } from '../engine/kinematics'
import { ALGO_BY_ID, EASE_OUT } from '../lib/theme'
import { ArmCanvas, type ArmPlanView, type ArmState, type ArmTool } from './ArmCanvas'
import { CountUp } from './CountUp'
import { IconArm, IconCircle, IconErase, IconPlay, IconRect, IconSkip, IconSwap } from './Icons'

const BASE: Point = { x: 500, y: 600 }
const THICKNESS = 14
// Matches the kinematics crate default; the 3-joint preset needs about 10,000 to succeed on every seed.
const ARM_MAX_ITER = 10000

const PRESETS = {
  3: { links: [180, 140, 90], a: [-1.2, -0.5, -0.3], b: [-1.94, -0.5, -0.3] },
  2: { links: [220, 180], a: [-1.0, -0.8], b: [-2.4, 0.6] },
} as const

const DEFAULT_OBSTACLES: Obstacle[] = [
  { kind: 'circle', x: 400, y: 250, r: 44 },
  { kind: 'rect', x: 120, y: 120, w: 150, h: 36 },
]

const TOOLS: { id: ArmTool; label: string; icon: () => React.ReactElement }[] = [
  { id: 'arm', label: 'Drag arm', icon: IconArm },
  { id: 'rect', label: 'Wall', icon: IconRect },
  { id: 'circle', label: 'Pillar', icon: IconCircle },
  { id: 'erase', label: 'Erase', icon: IconErase },
]

type Which = 'a' | 'b'

export function ArmMode({ toast }: { toast: (t: string, tone?: 'info' | 'danger') => void }) {
  const reduced = useReducedMotion() ?? false
  const [joints, setJoints] = useState<2 | 3>(3)
  const links = useMemo(() => [...PRESETS[joints].links], [joints])
  const [poseA, setPoseA] = useState<number[]>([...PRESETS[3].a])
  const [poseB, setPoseB] = useState<number[]>([...PRESETS[3].b])
  const [editing, setEditing] = useState<Which>('a')
  const [obstacles, setObstacles] = useState<Obstacle[]>(DEFAULT_OBSTACLES)
  const [tool, setTool] = useState<ArmTool>('arm')
  const [algo, setAlgo] = useState<ArmAlgorithm>('rrtstar')
  const [plan, setPlan] = useState<(TimedArmPlan & { key: string; algo: ArmAlgorithm }) | null>(null)
  const [phase, setPhase] = useState<{ p: 'explore' | 'trace' | 'move' | 'done'; f: number }>({ p: 'done', f: 1 })
  const [live, setLive] = useState<ArmState | null>(null)
  const [ikInfo, setIkInfo] = useState<{ reached: boolean; error: number }>({ reached: true, error: 0 })
  const [skip, setSkip] = useState(0)
  const [seed, setSeed] = useState(1)

  const arm = useMemo(() => ({ base: BASE, links }), [links])
  const pose = editing === 'a' ? poseA : poseB
  const ghost = editing === 'a' ? poseB : poseA
  const setPose = editing === 'a' ? setPoseA : setPoseB

  const solveIk = useCallback(
    (target: Point, seedAngles: number[]) => {
      try {
        const r = inverseKinematics({ ...arm, angles: seedAngles, target, maxIterations: 60, tolerance: 0.5 })
        setIkInfo((prev) => (prev.reached === r.reached && Math.abs(prev.error - r.error) < 0.5 ? prev : { reached: r.reached, error: r.error }))
        return { angles: r.angles, reached: r.reached }
      } catch {
        return { angles: seedAngles, reached: false }
      }
    },
    [arm],
  )
  const jointsFor = useCallback((a: number[]) => forwardKinematics(arm, a), [arm])
  const validFor = useCallback(
    (a: number[]) => {
      try {
        return armConfigValid({ ...arm, obstacles, linkThickness: THICKNESS, angles: a })
      } catch {
        return false
      }
    },
    [arm, obstacles],
  )

  const switchJoints = (n: 2 | 3) => {
    if (n === joints) return
    setJoints(n)
    setPoseA([...PRESETS[n].a])
    setPoseB([...PRESETS[n].b])
    setPlan(null)
  }

  const runPlan = () => {
    const ok = (a: number[]) => validFor(a)
    if (!ok(poseA) || !ok(poseB)) {
      toast(`Pose ${!ok(poseA) ? 'A' : 'B'} is touching a wall. Move it clear, then plan.`, 'danger')
      return
    }
    try {
      const r = planArm({
        ...arm,
        obstacles,
        linkThickness: THICKNESS,
        start: poseA,
        goal: poseB,
        algorithm: algo,
        seed,
        params: { maxIterations: ARM_MAX_ITER },
      })
      setPlan({ ...r, key: `${algo}-${Date.now()}`, algo })
      setPhase({ p: 'explore', f: 0 })
      if (!r.found) toast(`No collision-free motion found in ${r.iterations.toLocaleString('en-US')} iterations.`, 'danger')
    } catch (e) {
      toast((e as Error).message, 'danger')
    }
  }

  const planView: ArmPlanView | null = useMemo(
    () =>
      plan && {
        key: plan.key,
        color: ALGO_BY_ID[plan.algo].color,
        events: plan.events,
        path: plan.path,
        found: plan.found,
      },
    [plan],
  )

  const onEdit = useCallback(() => setPlan(null), [])
  const busy = !!plan && phase.p !== 'done'
  const valid = live?.valid ?? true
  const deg = (r: number) => ((r * 180) / Math.PI).toFixed(0)

  const statusText = busy
    ? phase.p === 'explore'
      ? `${ALGO_BY_ID[plan!.algo].name} is searching joint space`
      : phase.p === 'trace'
        ? 'Motion found, tracing the end effector'
        : 'Moving from pose A to pose B'
    : plan && !plan.found
      ? 'No collision-free motion between the two poses'
      : !valid
        ? 'Collision: a link is touching a wall'
        : !ikInfo.reached && tool === 'arm'
          ? 'Out of reach, the arm stretches as far as it can'
          : `Editing pose ${editing.toUpperCase()}. Drag to move the end effector.`
  const statusDanger = (!busy && plan && !plan.found) || (!busy && !valid)

  return (
    <div className="workbench">
      <div className="stage-col">
        <div className={`status-band${statusDanger ? ' is-danger' : ''}`} aria-live="polite">
          <AnimatePresence mode="popLayout" initial={false}>
            <motion.span
              key={statusText}
              className="status-text"
              initial={reduced ? { opacity: 0 } : { opacity: 0, transform: 'translateY(60%)', filter: 'blur(2px)' }}
              animate={{ opacity: 1, transform: 'translateY(0%)', filter: 'blur(0px)' }}
              exit={reduced ? { opacity: 0 } : { opacity: 0, transform: 'translateY(-60%)', filter: 'blur(2px)' }}
              transition={{ duration: 0.2, ease: EASE_OUT }}
            >
              {statusText}
            </motion.span>
          </AnimatePresence>
          <span className="status-count">{joints} joints</span>
        </div>
        <ArmCanvas
          base={BASE}
          links={links}
          thickness={THICKNESS}
          obstacles={obstacles}
          tool={tool}
          pose={pose}
          ghost={ghost}
          ghostLabel={`Pose ${editing === 'a' ? 'B' : 'A'}`}
          poseLabel={`Pose ${editing.toUpperCase()}`}
          plan={planView}
          skipToken={skip}
          reduced={reduced}
          solveIk={solveIk}
          jointsFor={jointsFor}
          validFor={validFor}
          onPose={setPose}
          onObstacles={setObstacles}
          onEdit={onEdit}
          onReject={(m) => toast(m, 'danger')}
          onPlanPhase={(p, f) => setPhase({ p, f })}
          onLive={setLive}
        />
        <div className="tools" role="toolbar" aria-label="Arm tools">
          {TOOLS.map((t) => (
            <button key={t.id} type="button" className={`tool${tool === t.id ? ' is-on' : ''}`} aria-pressed={tool === t.id} onClick={() => setTool(t.id)}>
              {tool === t.id && <motion.span layoutId="arm-tool-pill" className="tool-pill" transition={{ type: 'spring', duration: 0.35, bounce: 0.15 }} />}
              <span className="tool-inner">
                <t.icon />
                {t.label}
              </span>
            </button>
          ))}
        </div>
      </div>

      <aside className="control-col">
        <div className="actions">
          <button type="button" className="btn-primary" onClick={runPlan} disabled={busy}>
            <IconPlay />
            Plan A to B
          </button>
          <AnimatePresence initial={false}>
            {busy && (
              <motion.button
                type="button"
                className="btn-ghost"
                onClick={() => setSkip((s) => s + 1)}
                initial={{ opacity: 0, transform: 'scale(0.95)' }}
                animate={{ opacity: 1, transform: 'scale(1)' }}
                exit={{ opacity: 0, transform: 'scale(0.95)', transition: { duration: 0.12 } }}
                transition={{ duration: 0.18, ease: EASE_OUT }}
              >
                <IconSkip />
                Skip to result
              </motion.button>
            )}
          </AnimatePresence>
        </div>

        <div className="group">
          <h2 className="h-small">Pose</h2>
          <div className="segmented" role="tablist" aria-label="Pose being edited">
            {(['a', 'b'] as const).map((w) => (
              <button
                key={w}
                type="button"
                role="tab"
                aria-selected={editing === w}
                className={`seg${editing === w ? ' is-on' : ''}`}
                onClick={() => {
                  setEditing(w)
                  setPlan(null)
                  setTool('arm')
                }}
              >
                {editing === w && <motion.span layoutId="pose-seg" className="seg-pill" transition={{ type: 'spring', duration: 0.35, bounce: 0.15 }} />}
                <span className="seg-label">Pose {w.toUpperCase()}</span>
              </button>
            ))}
            <button
              type="button"
              className="seg seg-icon"
              aria-label="Swap poses"
              onClick={() => {
                setPoseA(poseB)
                setPoseB(poseA)
                setPlan(null)
              }}
            >
              <span className="seg-label">
                <IconSwap />
              </span>
            </button>
          </div>
          <div className="joint-readout">
            {(live?.angles ?? pose).map((a, i) => (
              <span key={i} className="joint">
                <span className="joint-k">J{i + 1}</span>
                <span className="num">{deg(a)}&deg;</span>
              </span>
            ))}
            <span className={`joint joint-state${valid ? '' : ' is-danger'}`}>
              <span className="joint-k">State</span>
              <span>{valid ? 'Clear' : 'Collision'}</span>
            </span>
          </div>
        </div>

        <div className="group">
          <h2 className="h-small">Arm</h2>
          <div className="segmented" role="tablist" aria-label="Number of joints">
            {([2, 3] as const).map((n) => (
              <button key={n} type="button" role="tab" aria-selected={joints === n} className={`seg${joints === n ? ' is-on' : ''}`} onClick={() => switchJoints(n)}>
                {joints === n && <motion.span layoutId="joint-seg" className="seg-pill" transition={{ type: 'spring', duration: 0.35, bounce: 0.15 }} />}
                <span className="seg-label">{n} joints</span>
              </button>
            ))}
          </div>
        </div>

        <fieldset className="algos">
          <legend className="h-small">Joint-space planner</legend>
          {(['rrt', 'rrtstar'] as const).map((id) => {
            const a = ALGO_BY_ID[id]
            return (
              <label key={id} className={`algo${algo === id ? ' is-on' : ''}`} style={{ ['--livery' as string]: a.color }}>
                <input type="radio" name="arm-algo" checked={algo === id} onChange={() => setAlgo(id)} />
                <span className="algo-dot" />
                <span className="algo-text">
                  <span className="algo-name">{a.name}</span>
                  <span className="algo-blurb">{a.blurb}</span>
                </span>
              </label>
            )
          })}
          <div className="plan-row">
            <div className="seed">
              <span className="seed-label">Seed</span>
              <button type="button" className="btn-ghost icon-only" aria-label="Previous seed" disabled={seed <= 1} onClick={() => setSeed((s) => Math.max(1, s - 1))}>
                <svg width="16" height="16" viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" aria-hidden><path d="M5 10h10" /></svg>
              </button>
              <span className="seed-val num">{seed}</span>
              <button type="button" className="btn-ghost icon-only" aria-label="Next seed" onClick={() => setSeed((s) => s + 1)}>
                <svg width="16" height="16" viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" aria-hidden><path d="M5 10h10M10 5v10" /></svg>
              </button>
            </div>
          </div>
        </fieldset>

        <div className="tower-wrap">
          <h2 className="h-small">Last plan</h2>
          {plan ? (
            <dl className="stats" style={{ ['--livery' as string]: ALGO_BY_ID[plan.algo].color }}>
              <div>
                <dt>Planner</dt>
                <dd className="who">
                  <span className="swatch" />
                  <span className="who-name">{ALGO_BY_ID[plan.algo].name}</span>
                </dd>
              </div>
              <div>
                <dt>Time ms</dt>
                <dd><CountUp value={plan.timeMs} decimals={2} /></dd>
              </div>
              <div>
                <dt>Nodes</dt>
                <dd><CountUp value={plan.nodesExplored} /></dd>
              </div>
              <div>
                <dt>Waypoints</dt>
                <dd>{plan.found ? <CountUp value={plan.path.length} /> : <span className="dnf">DNF</span>}</dd>
              </div>
            </dl>
          ) : (
            <p className="tower-empty">Set pose A and pose B, then plan a collision-free motion between them. The tree is drawn where the end effector would be.</p>
          )}
        </div>
      </aside>
    </div>
  )
}
