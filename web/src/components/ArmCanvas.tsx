import { useEffect, useRef } from 'react'
import type { Obstacle, Point } from '../engine/types'
import { COLORS, WORLD_H, WORLD_W, easeInOutCubic, easeOutBack, easeOutExpo } from '../lib/theme'
import { clamp, dist, fkLocal, normRect, pointInObstacle } from '../lib/geometry'
import { drawField, drawObstacle, fitCanvas, label, mixHex, rgba, type Surface } from '../lib/draw'

export type ArmTool = 'arm' | 'rect' | 'circle' | 'erase'

export interface ArmPlanView {
  key: string
  color: string
  events: { q: number[]; pq: number[]; k: 'expand' | 'rewire' }[]
  path: number[][]
  found: boolean
}

export interface ArmState {
  angles: number[]
  joints: Point[]
  valid: boolean
}

interface Props {
  base: Point
  links: number[]
  thickness: number
  obstacles: Obstacle[]
  tool: ArmTool
  /** The pose the visitor is editing (live, follows drag). */
  pose: number[]
  /** The other pose, drawn as a ghost. */
  ghost: number[] | null
  ghostLabel: string
  poseLabel: string
  plan: ArmPlanView | null
  skipToken: number
  reduced: boolean
  solveIk: (target: Point, seed: number[]) => { angles: number[]; reached: boolean }
  jointsFor: (angles: number[]) => Point[]
  validFor: (angles: number[]) => boolean
  onPose: (angles: number[]) => void
  onObstacles: (next: Obstacle[]) => void
  onEdit: () => void
  onReject: (msg: string) => void
  onPlanPhase: (phase: 'explore' | 'trace' | 'move' | 'done', frac: number) => void
  onLive: (s: ArmState) => void
}

type Drag =
  | { kind: 'arm' }
  | { kind: 'rect'; x0: number; y0: number; x1: number; y1: number }
  | { kind: 'circle'; x0: number; y0: number; r: number }

export function ArmCanvas(props: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const propsRef = useRef(props)
  propsRef.current = props

  useEffect(() => {
    const canvas = canvasRef.current!
    const ctx = canvas.getContext('2d')!
    let surface: Surface = fitCanvas(canvas, WORLD_W, WORLD_H)
    let raf = 0
    const mountT = performance.now()
    let last = mountT
    const px = () => 1 / (surface.scale / surface.dpr)

    const births = new WeakMap<Obstacle, number>()
    let prevObs: Obstacle[] = []
    const ghosts: { o: Obstacle; t: number }[] = []

    // Displayed angles chase the solved pose with a critically damped spring,
    // so joints follow the finger smoothly instead of snapping.
    let shown = [...propsRef.current.pose]
    let vel = shown.map(() => 0)
    let target: Point | null = null
    let springTarget: Point | null = null
    let drag: Drag | null = null
    let activePointer: number | null = null
    let hover: Point | null = null
    let danger = 0
    let lastValidKey = ''
    let lastValid = true
    let rejectPulse = 0

    // plan playback
    let planKey = ''
    let planT0 = 0
    let planPhase: 'explore' | 'trace' | 'move' | 'done' = 'done'
    let planPhaseT = 0
    let treeCanvas: HTMLCanvasElement | null = null
    let played = 0
    let projected: { x: number; y: number; px: number; py: number; k: string }[] = []
    let eePath: Point[] = []
    let lastSkip = propsRef.current.skipToken
    let lastReport = 0

    function resetPlan(now: number) {
      const P = propsRef.current
      const plan = P.plan
      planKey = plan?.key ?? ''
      treeCanvas = null
      played = 0
      projected = []
      eePath = []
      if (!plan) {
        planPhase = 'done'
        return
      }
      projected = plan.events.map((e) => {
        const a = fkLocal(P.base, P.links, e.q)
        const b = fkLocal(P.base, P.links, e.pq)
        const ea = a[a.length - 1]
        const eb = b[b.length - 1]
        return { x: ea.x, y: ea.y, px: eb.x, py: eb.y, k: e.k }
      })
      eePath = plan.path.map((q) => {
        const j = fkLocal(P.base, P.links, q)
        return j[j.length - 1]
      })
      treeCanvas = document.createElement('canvas')
      treeCanvas.width = canvas.width
      treeCanvas.height = canvas.height
      planT0 = now + 100
      planPhase = 'explore'
      planPhaseT = now
      if (P.reduced) {
        flushTree(projected.length)
        planPhase = plan.found ? 'done' : 'done'
        P.onPlanPhase('done', 1)
      }
    }

    function flushTree(to: number) {
      if (!treeCanvas || !propsRef.current.plan) return
      const c = treeCanvas.getContext('2d')!
      c.setTransform(surface.scale, 0, 0, surface.scale, 0, 0)
      c.lineWidth = 1.1 * px()
      c.lineCap = 'round'
      c.strokeStyle = rgba(propsRef.current.plan.color, 0.55)
      c.beginPath()
      for (let i = played; i < to; i++) {
        const e = projected[i]
        c.moveTo(e.px, e.py)
        c.lineTo(e.x, e.y)
      }
      c.stroke()
      played = to
    }

    const moveDuration = () => clamp((propsRef.current.plan?.path.length ?? 0) * 22, 1200, 4200)
    const exploreDuration = () => clamp(projected.length * 0.9, 900, 2600)

    function frame(now: number) {
      raf = requestAnimationFrame(frame)
      const dt = Math.min(1 / 30, (now - last) / 1000)
      last = now
      const P = propsRef.current

      if ((P.plan?.key ?? '') !== planKey) resetPlan(now)
      if (P.skipToken !== lastSkip) {
        lastSkip = P.skipToken
        if (P.plan && planPhase !== 'done') {
          flushTree(projected.length)
          planPhase = 'done'
          P.onPlanPhase('done', 1)
        }
      }

      // obstacles
      let batch = 0
      for (const o of P.obstacles) if (!births.has(o)) births.set(o, P.reduced ? -1e9 : now + Math.min(batch++ * 30, 400))
      if (prevObs !== P.obstacles) {
        const cur = new Set(P.obstacles)
        for (const o of prevObs) if (!cur.has(o) && !P.reduced) ghosts.push({ o, t: now })
        prevObs = P.obstacles
      }

      // IK drag: spring the target point, then solve every frame
      if (drag?.kind === 'arm' && target) {
        if (!springTarget || P.reduced) springTarget = { ...target }
        else {
          const k = 1 - Math.exp(-dt * 26)
          springTarget = { x: springTarget.x + (target.x - springTarget.x) * k, y: springTarget.y + (target.y - springTarget.y) * k }
        }
        const sol = P.solveIk(springTarget, P.pose)
        P.onPose(sol.angles)
      }

      // pick the pose to display: plan playback overrides the live pose
      let goalAngles = P.pose
      const plan = P.plan
      if (plan && plan.found && (planPhase === 'move' || planPhase === 'done') && plan.path.length) {
        if (planPhase === 'move') {
          const t = easeInOutCubic(clamp((now - planPhaseT) / moveDuration(), 0, 1))
          const f = t * (plan.path.length - 1)
          const i = Math.floor(f)
          const u = f - i
          const a = plan.path[i]
          const b = plan.path[Math.min(i + 1, plan.path.length - 1)]
          goalAngles = a.map((v, j) => v + (b[j] - v) * u)
        } else goalAngles = plan.path[plan.path.length - 1]
      } else if (plan && (planPhase === 'explore' || planPhase === 'trace') && plan.path.length) {
        goalAngles = plan.path[0]
      }

      const playing = plan && planPhase === 'move'
      if (P.reduced || playing || drag?.kind === 'arm') {
        shown = [...goalAngles]
        vel = shown.map(() => 0)
      } else {
        const stiff = 260
        const damp = 2 * Math.sqrt(stiff)
        if (shown.length !== goalAngles.length) {
          shown = [...goalAngles]
          vel = shown.map(() => 0)
        }
        for (let i = 0; i < shown.length; i++) {
          let d = goalAngles[i] - shown[i]
          if (d > Math.PI) d -= Math.PI * 2
          if (d < -Math.PI) d += Math.PI * 2
          vel[i] += (d * stiff - vel[i] * damp) * dt
          shown[i] += vel[i] * dt
        }
      }

      const joints = P.jointsFor(shown)
      const vkey = shown.map((a) => a.toFixed(3)).join(',') + '|' + P.obstacles.length + '|' + P.links.length
      if (vkey !== lastValidKey) {
        lastValidKey = vkey
        lastValid = P.validFor(shown)
        P.onLive({ angles: shown, joints, valid: lastValid })
      }
      danger += ((lastValid ? 0 : 1) - danger) * Math.min(1, dt * 14)

      // plan phases
      if (plan && planPhase === 'explore') {
        const to = Math.min(projected.length, Math.floor(((now - planT0) / exploreDuration()) * projected.length))
        if (to > played) flushTree(to)
        if (now - lastReport > 90) {
          lastReport = now
          P.onPlanPhase('explore', projected.length ? played / projected.length : 1)
        }
        if (played >= projected.length && now > planT0) {
          planPhase = plan.found ? 'trace' : 'done'
          planPhaseT = now
          P.onPlanPhase(planPhase, plan.found ? 0 : 1)
        }
      } else if (plan && planPhase === 'trace') {
        if (now - planPhaseT > 900) {
          planPhase = 'move'
          planPhaseT = now
          P.onPlanPhase('move', 0)
        }
      } else if (plan && planPhase === 'move') {
        if (now - lastReport > 90) {
          lastReport = now
          P.onPlanPhase('move', clamp((now - planPhaseT) / moveDuration(), 0, 1))
        }
        if (now - planPhaseT >= moveDuration()) {
          planPhase = 'done'
          P.onPlanPhase('done', 1)
        }
      }

      // ---- draw ----
      const unit = px()
      ctx.setTransform(1, 0, 0, 1, 0, 0)
      ctx.clearRect(0, 0, canvas.width, canvas.height)
      ctx.setTransform(surface.scale, 0, 0, surface.scale, 0, 0)
      const reveal = P.reduced ? 1 : easeOutExpo(clamp((now - mountT) / 1100, 0, 1))
      drawField(ctx, WORLD_W, WORLD_H, 40, 1.1 * unit, reveal)

      // reach envelope
      const reach = P.links.reduce((a, b) => a + b, 0)
      ctx.setLineDash([4 * unit, 6 * unit])
      ctx.strokeStyle = 'rgba(236,238,241,0.14)'
      ctx.lineWidth = unit
      ctx.beginPath()
      ctx.arc(P.base.x, P.base.y, reach * reveal, Math.PI, Math.PI * 2)
      ctx.stroke()
      ctx.setLineDash([])

      for (let i = ghosts.length - 1; i >= 0; i--) {
        const g = ghosts[i]
        const t = (now - g.t) / 170
        if (t >= 1) {
          ghosts.splice(i, 1)
          continue
        }
        drawObstacle(ctx, g.o, { scale: 1 - 0.1 * t, alpha: 1 - t, px: unit, danger: 1 - t })
      }
      for (const o of P.obstacles) {
        const b = births.get(o) ?? 0
        if (now < b) continue
        const t = clamp((now - b) / 340, 0, 1)
        const he = P.tool === 'erase' && hover && pointInObstacle(o, hover) ? 1 : 0
        drawObstacle(ctx, o, { scale: 0.84 + 0.16 * easeOutBack(t), alpha: easeOutExpo(t), px: unit, danger: he })
      }

      // tree
      if (treeCanvas && plan) {
        ctx.setTransform(1, 0, 0, 1, 0, 0)
        ctx.globalAlpha = planPhase === 'move' || planPhase === 'done' ? 0.6 : 1
        ctx.drawImage(treeCanvas, 0, 0)
        ctx.globalAlpha = 1
        ctx.setTransform(surface.scale, 0, 0, surface.scale, 0, 0)
        if (planPhase === 'explore' && played > 0) {
          const h = projected[played - 1]
          ctx.fillStyle = COLORS.white
          ctx.beginPath()
          ctx.arc(h.x, h.y, 2.8 * unit, 0, Math.PI * 2)
          ctx.fill()
        }
        // end-effector path
        if (plan.found && planPhase !== 'explore' && eePath.length > 1) {
          let frac = 1
          if (planPhase === 'trace') frac = easeInOutCubic(clamp((now - planPhaseT) / 800, 0, 1))
          const n = Math.max(2, Math.floor(eePath.length * frac))
          ctx.lineCap = 'round'
          ctx.lineJoin = 'round'
          ctx.strokeStyle = 'rgba(11,12,14,0.85)'
          ctx.lineWidth = 6 * unit
          ctx.beginPath()
          eePath.slice(0, n).forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)))
          ctx.stroke()
          ctx.strokeStyle = plan.color
          ctx.lineWidth = 2.6 * unit
          ctx.stroke()
        }
      }

      // ghost pose
      if (P.ghost && !(plan && planPhase !== 'done')) {
        const gj = fkLocal(P.base, P.links, P.ghost)
        drawArm(gj, P.thickness, 'rgba(236,238,241,0.18)', 'rgba(236,238,241,0.35)', unit, true)
        const ee = gj[gj.length - 1]
        label(ctx, P.ghostLabel, ee.x, ee.y - 16, unit, COLORS.inkSoft)
      }

      // live arm
      const body = mixHex('#E9ECF0', COLORS.danger, danger)
      drawArm(joints, P.thickness, body, mixHex('#34D1C4', COLORS.danger, danger), unit, false)
      const ee = joints[joints.length - 1]
      const grab = drag?.kind === 'arm' ? 1 : hover && dist(hover, ee) < 30 ? 0.6 : 0
      ctx.strokeStyle = rgba('#34D1C4', 0.35 + 0.5 * grab)
      ctx.lineWidth = 1.5 * unit
      ctx.beginPath()
      ctx.arc(ee.x, ee.y, 16 + grab * 4 + (P.reduced ? 0 : Math.sin(now / 400) * 1.2), 0, Math.PI * 2)
      ctx.stroke()
      label(ctx, P.poseLabel, ee.x, ee.y - 24, unit, COLORS.ink)
      if (drag?.kind === 'arm' && target) {
        ctx.strokeStyle = 'rgba(247,248,250,0.5)'
        ctx.setLineDash([3 * unit, 4 * unit])
        ctx.beginPath()
        ctx.moveTo(ee.x, ee.y)
        ctx.lineTo(target.x, target.y)
        ctx.stroke()
        ctx.setLineDash([])
        ctx.fillStyle = COLORS.white
        ctx.beginPath()
        ctx.arc(target.x, target.y, 3 * unit, 0, Math.PI * 2)
        ctx.fill()
      }

      // base plate
      ctx.fillStyle = '#2B3038'
      ctx.beginPath()
      ctx.roundRect(P.base.x - 46, P.base.y + 6, 92, 22, 4)
      ctx.fill()

      // draft obstacle
      if (drag && drag.kind !== 'arm') {
        const o: Obstacle = drag.kind === 'rect' ? normRect(drag.x0, drag.y0, drag.x1, drag.y1) : { kind: 'circle', x: drag.x0, y: drag.y0, r: drag.r }
        drawObstacle(ctx, o, { scale: 1, alpha: 0.85, px: unit })
      }

      if (rejectPulse > 0) {
        rejectPulse = Math.max(0, rejectPulse - dt * 3)
        ctx.strokeStyle = rgba(COLORS.danger, rejectPulse)
        ctx.lineWidth = 3 * unit
        ctx.strokeRect(1.5 * unit, 1.5 * unit, WORLD_W - 3 * unit, WORLD_H - 3 * unit)
      }
    }

    function drawArm(j: Point[], thickness: number, body: string, joint: string, unit: number, ghost: boolean) {
      ctx.lineCap = 'round'
      ctx.lineJoin = 'round'
      if (!ghost) {
        ctx.strokeStyle = 'rgba(0,0,0,0.35)'
        ctx.lineWidth = thickness + 6 * unit
        ctx.beginPath()
        j.forEach((p, i) => (i ? ctx.lineTo(p.x + 3, p.y + 5) : ctx.moveTo(p.x + 3, p.y + 5)))
        ctx.stroke()
      }
      ctx.strokeStyle = body
      ctx.lineWidth = thickness
      ctx.beginPath()
      j.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)))
      ctx.stroke()
      j.forEach((p, i) => {
        ctx.fillStyle = ghost ? 'rgba(18,20,23,0.6)' : '#121417'
        ctx.beginPath()
        ctx.arc(p.x, p.y, i === j.length - 1 ? 6 : thickness * 0.62, 0, Math.PI * 2)
        ctx.fill()
        ctx.strokeStyle = joint
        ctx.lineWidth = 2.2 * unit
        ctx.stroke()
      })
    }

    raf = requestAnimationFrame(frame)

    const toWorld = (e: PointerEvent): Point => {
      const rect = canvas.getBoundingClientRect()
      return {
        x: clamp(((e.clientX - rect.left) / rect.width) * WORLD_W, 0, WORLD_W),
        y: clamp(((e.clientY - rect.top) / rect.height) * WORLD_H, 0, WORLD_H),
      }
    }

    function onDown(e: PointerEvent) {
      if (activePointer !== null) return
      const P = propsRef.current
      const p = toWorld(e)
      if (P.tool === 'arm') {
        drag = { kind: 'arm' }
        target = p
        springTarget = null
        P.onEdit()
      } else if (P.tool === 'rect') drag = { kind: 'rect', x0: p.x, y0: p.y, x1: p.x, y1: p.y }
      else if (P.tool === 'circle') drag = { kind: 'circle', x0: p.x, y0: p.y, r: 0 }
      else if (P.tool === 'erase') {
        for (let i = P.obstacles.length - 1; i >= 0; i--) {
          if (pointInObstacle(P.obstacles[i], p, e.pointerType === 'touch' ? 8 : 2)) {
            P.onEdit()
            P.onObstacles(P.obstacles.filter((_, k) => k !== i))
            break
          }
        }
        return
      }
      activePointer = e.pointerId
      try {
        canvas.setPointerCapture(e.pointerId)
      } catch {
        // synthetic or already-released pointers cannot be captured
      }
      e.preventDefault()
    }

    function onMove(e: PointerEvent) {
      const p = toWorld(e)
      hover = p
      const P = propsRef.current
      canvas.style.cursor = P.tool === 'arm' ? (drag ? 'grabbing' : 'grab') : P.tool === 'erase' ? 'pointer' : 'crosshair'
      if (e.pointerId !== activePointer || !drag) return
      if (drag.kind === 'arm') target = p
      else if (drag.kind === 'rect') {
        drag.x1 = p.x
        drag.y1 = p.y
      } else drag.r = dist(p, { x: drag.x0, y: drag.y0 })
    }

    function onUp(e: PointerEvent) {
      if (e.pointerId !== activePointer) return
      activePointer = null
      const P = propsRef.current
      if (drag?.kind === 'arm' && target) {
        // settle exactly on the release point rather than the spring-lagged one
        P.onPose(P.solveIk(target, P.pose).angles)
      }
      if (drag && drag.kind !== 'arm') {
        let o: Obstacle = drag.kind === 'rect' ? normRect(drag.x0, drag.y0, drag.x1, drag.y1) : { kind: 'circle', x: drag.x0, y: drag.y0, r: drag.r }
        if (o.kind === 'rect' && o.w < 8 && o.h < 8) o = { kind: 'rect', x: o.x - 30, y: o.y - 30, w: 60, h: 60 }
        if (o.kind === 'circle' && o.r < 6) o = { ...o, r: 30 }
        if (pointInObstacle(o, P.base, 30)) {
          rejectPulse = 1
          P.onReject('Walls cannot sit on the arm base.')
        } else {
          P.onEdit()
          P.onObstacles([...P.obstacles, o])
        }
      }
      drag = null
      target = null
      springTarget = null
    }

    const onLeave = () => {
      hover = null
    }

    canvas.addEventListener('pointerdown', onDown)
    canvas.addEventListener('pointermove', onMove)
    canvas.addEventListener('pointerup', onUp)
    canvas.addEventListener('pointercancel', onUp)
    canvas.addEventListener('pointerleave', onLeave)
    const ro = new ResizeObserver(() => {
      surface = fitCanvas(canvas, WORLD_W, WORLD_H)
      if (treeCanvas) {
        treeCanvas.width = canvas.width
        treeCanvas.height = canvas.height
        const n = played
        played = 0
        flushTree(n)
      }
    })
    ro.observe(canvas.parentElement!)
    return () => {
      cancelAnimationFrame(raf)
      ro.disconnect()
      canvas.removeEventListener('pointerdown', onDown)
      canvas.removeEventListener('pointermove', onMove)
      canvas.removeEventListener('pointerup', onUp)
      canvas.removeEventListener('pointercancel', onUp)
      canvas.removeEventListener('pointerleave', onLeave)
    }
  }, [])

  return (
    <div className="stage">
      <canvas
        ref={canvasRef}
        className="stage-canvas"
        role="img"
        aria-label="Arm workspace. Drag anywhere to pull the end effector; the joints follow by inverse kinematics."
      />
    </div>
  )
}
