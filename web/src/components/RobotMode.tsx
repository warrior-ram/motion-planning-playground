import { AnimatePresence, motion, useReducedMotion } from 'motion/react'
import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import type { Algorithm, BenchMap, Obstacle, Point } from '../engine/types'
import { runPlan, type TimedPlan } from '../engine/planner'
import { ALGOS, ALGO_BY_ID, EASE_OUT, ROBOT_RADIUS, WORLD_H, WORLD_W } from '../lib/theme'
import { RobotCanvas, type RunProgress, type RunView, type Tool } from './RobotCanvas'
import { TimingTower, type TowerRow } from './TimingTower'
import { PresetRail } from './PresetRail'
import { IconCircle, IconErase, IconFlag, IconMove, IconPlay, IconRect, IconReset, IconSkip } from './Icons'

// Matches the planner crate's default, so the page and bench/results.md use the same budget.
const MAX_ITER = 20000
const GRID = 10

const TOOLS: { id: Tool; label: string; icon: () => React.ReactElement }[] = [
  { id: 'rect', label: 'Wall', icon: IconRect },
  { id: 'circle', label: 'Pillar', icon: IconCircle },
  { id: 'move', label: 'Move', icon: IconMove },
  { id: 'erase', label: 'Erase', icon: IconErase },
]

interface Props {
  maps: BenchMap[]
  toast: (text: string, tone?: 'info' | 'danger') => void
}

export function RobotMode({ maps, toast }: Props) {
  const reduced = useReducedMotion() ?? false
  const first = maps[1] ?? maps[0]
  const [obstacles, setObstacles] = useState<Obstacle[]>(first?.obstacles ?? [])
  const [start, setStart] = useState<Point>(first?.start ?? { x: 80, y: 320 })
  const [goal, setGoal] = useState<Point>(first?.goal ?? { x: 920, y: 320 })
  const [mapId, setMapId] = useState<string | null>(first?.id ?? null)
  const [tool, setTool] = useState<Tool>('rect')
  const [algo, setAlgo] = useState<Algorithm>('rrtstar')
  const [seed, setSeed] = useState(1)
  const [runs, setRuns] = useState<RunView[]>([])
  const [race, setRace] = useState(false)
  const [progress, setProgress] = useState<Record<string, RunProgress>>({})
  const [skipToken, setSkip] = useState(0)
  const runCounter = useRef(0)
  const autoRaced = useRef(false)

  const clearRuns = useCallback(() => {
    setRuns((r) => (r.length ? [] : r))
    setMapId(null)
  }, [])

  const plan = useCallback(
    (algos: Algorithm[]) => {
      const out: RunView[] = []
      for (const a of algos) {
        let result: TimedPlan
        try {
          result = runPlan({
            algorithm: a,
            width: WORLD_W,
            height: WORLD_H,
            obstacles,
            start,
            goal,
            robotRadius: ROBOT_RADIUS,
            seed,
            params: { gridSize: GRID, maxIterations: MAX_ITER },
          })
        } catch (e) {
          toast(`${ALGO_BY_ID[a].name}: ${(e as Error).message}`, 'danger')
          continue
        }
        out.push({ key: `${a}-${++runCounter.current}`, algo: a, result })
      }
      setProgress({})
      setRace(algos.length > 1)
      setRuns(out)
    },
    [obstacles, start, goal, seed, toast],
  )

  // The page opens on a race already running.
  useEffect(() => {
    if (autoRaced.current) return
    const t = setTimeout(() => {
      autoRaced.current = true
      plan(['astar', 'rrt', 'rrtstar'])
    }, reduced ? 0 : 700)
    return () => clearTimeout(t)
  }, [plan, reduced])

  const loadMap = (m: BenchMap) => {
    setObstacles(m.obstacles)
    setStart(m.start)
    setGoal(m.goal)
    setRuns([])
    setMapId(m.id)
  }

  const onProgress = useCallback((p: RunProgress) => {
    setProgress((prev) => {
      const cur = prev[p.key]
      if (cur && cur.phase === p.phase && Math.abs(cur.frac - p.frac) < 0.01) return prev
      return { ...prev, [p.key]: p }
    })
  }, [])

  const rows: TowerRow[] = useMemo(
    () =>
      runs.map((r) => {
        const p = progress[r.key]
        return {
          algo: r.algo,
          found: r.result.found,
          timeMs: r.result.timeMs,
          nodes: r.result.nodesExplored,
          length: r.result.pathLength,
          iterations: r.result.iterations,
          phase: p?.phase ?? 'explore',
          frac: p?.frac ?? 0,
        }
      }),
    [runs, progress],
  )

  const status = useMemo(() => {
    if (!runs.length) return { key: 'idle', text: 'Draw walls, drag the start or the goal, then plan.' }
    const live = rows.filter((r) => r.phase !== 'arrived' && r.phase !== 'failed')
    if (live.length) {
      const exploring = live.find((r) => r.phase === 'explore')
      if (exploring) return { key: `ex-${exploring.algo}`, text: `${ALGO_BY_ID[exploring.algo].name} is exploring the map` }
      return { key: 'drive', text: 'Path found, driving to the goal' }
    }
    const ok = rows.filter((r) => r.found)
    if (!ok.length) return { key: 'fail', text: 'No planner reached the goal on this map', tone: 'danger' as const }
    if (rows.length === 1) {
      const r = rows[0]
      return { key: 'one', text: `${ALGO_BY_ID[r.algo].name} reached the goal in ${r.timeMs.toFixed(2)} ms` }
    }
    const win = [...ok].sort((a, b) => a.length - b.length)[0]
    return { key: `win-${win.algo}`, text: `${ALGO_BY_ID[win.algo].name} found the shortest path` }
  }, [runs.length, rows])

  const busy = rows.some((r) => r.phase !== 'arrived' && r.phase !== 'failed')
  const stochastic = algo !== 'astar' || race

  return (
    <div className="workbench">
      <div className="stage-col">
        <div className={`status-band${'tone' in status && status.tone === 'danger' ? ' is-danger' : ''}`} aria-live="polite">
          <AnimatePresence mode="popLayout" initial={false}>
            <motion.span
              key={status.key}
              className="status-text"
              initial={reduced ? { opacity: 0 } : { opacity: 0, transform: 'translateY(60%)', filter: 'blur(2px)' }}
              animate={{ opacity: 1, transform: 'translateY(0%)', filter: 'blur(0px)' }}
              exit={reduced ? { opacity: 0 } : { opacity: 0, transform: 'translateY(-60%)', filter: 'blur(2px)' }}
              transition={{ duration: 0.2, ease: EASE_OUT }}
            >
              {status.text}
            </motion.span>
          </AnimatePresence>
          <span className="status-count">
            {obstacles.length} {obstacles.length === 1 ? 'wall' : 'walls'}
          </span>
        </div>
        <RobotCanvas
          obstacles={obstacles}
          start={start}
          goal={goal}
          tool={tool}
          gridSize={GRID}
          runs={runs}
          race={race}
          skipToken={skipToken}
          reduced={reduced}
          onObstacles={setObstacles}
          onStart={setStart}
          onGoal={setGoal}
          onEdit={clearRuns}
          onReject={(m) => toast(m, 'danger')}
          onProgress={onProgress}
        />
        <div className="tools" role="toolbar" aria-label="Drawing tools">
          {TOOLS.map((t) => (
            <button
              key={t.id}
              type="button"
              className={`tool${tool === t.id ? ' is-on' : ''}`}
              aria-pressed={tool === t.id}
              onClick={() => setTool(t.id)}
            >
              {tool === t.id && <motion.span layoutId="tool-pill" className="tool-pill" transition={{ type: 'spring', duration: 0.35, bounce: 0.15 }} />}
              <span className="tool-inner">
                <t.icon />
                {t.label}
              </span>
            </button>
          ))}
          <span className="tools-sep" />
          <button
            type="button"
            className="btn-ghost"
            disabled={!obstacles.length}
            onClick={() => {
              setObstacles([])
              clearRuns()
            }}
          >
            <IconReset />
            Clear walls
          </button>
        </div>
      </div>

      <aside className="control-col">
        <div className="actions">
          <button type="button" className="btn-primary" onClick={() => plan(['astar', 'rrt', 'rrtstar'])}>
            <IconFlag />
            Race all three
          </button>
          <AnimatePresence initial={false}>
            {busy && (
              <motion.button
                type="button"
                className="btn-ghost"
                onClick={() => setSkip((s) => s + 1)}
                initial={reduced ? { opacity: 0 } : { opacity: 0, transform: 'scale(0.95)' }}
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

        <fieldset className="algos">
          <legend className="h-small">Planner</legend>
          {ALGOS.map((a) => (
            <label key={a.id} className={`algo${algo === a.id ? ' is-on' : ''}`} style={{ ['--livery' as string]: a.color }}>
              <input type="radio" name="algo" value={a.id} checked={algo === a.id} onChange={() => setAlgo(a.id)} />
              <span className="algo-dot" />
              <span className="algo-text">
                <span className="algo-name">{a.name}</span>
                <span className="algo-blurb">{a.blurb}</span>
              </span>
            </label>
          ))}
          <div className="plan-row">
            <button type="button" className="btn-secondary" onClick={() => plan([algo])}>
              <IconPlay />
              Plan with {ALGO_BY_ID[algo].name}
            </button>
            <div className={`seed${stochastic ? '' : ' is-muted'}`}>
              <span className="seed-label">Seed</span>
              <button type="button" className="btn-ghost icon-only" aria-label="Previous seed" disabled={seed <= 1} onClick={() => setSeed((s) => Math.max(1, s - 1))}>
                <svg width="16" height="16" viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" aria-hidden><path d="M5 10h10" /></svg>
              </button>
              <AnimatePresence mode="popLayout" initial={false}>
                <motion.span
                  key={seed}
                  className="seed-val num"
                  initial={reduced ? { opacity: 0 } : { opacity: 0, transform: 'translateY(-8px)' }}
                  animate={{ opacity: 1, transform: 'translateY(0px)' }}
                  exit={{ opacity: 0, transform: 'translateY(8px)', transition: { duration: 0.1 } }}
                  transition={{ duration: 0.18, ease: EASE_OUT }}
                >
                  {seed}
                </motion.span>
              </AnimatePresence>
              <button type="button" className="btn-ghost icon-only" aria-label="Next seed" onClick={() => setSeed((s) => s + 1)}>
                <svg width="16" height="16" viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" aria-hidden><path d="M5 10h10M10 5v10" /></svg>
              </button>
            </div>
          </div>
        </fieldset>

        <div className="tower-wrap">
          <h2 className="h-small">Timing</h2>
          {rows.length ? (
            <TimingTower rows={rows} maxIterations={MAX_ITER} />
          ) : (
            <p className="tower-empty">Results land here: planning time, nodes explored and path length for each planner. Purple marks the best in a column.</p>
          )}
        </div>
      </aside>

      <div className="rail-col">
        <PresetRail maps={maps} activeId={mapId} onPick={loadMap} />
      </div>
    </div>
  )
}
