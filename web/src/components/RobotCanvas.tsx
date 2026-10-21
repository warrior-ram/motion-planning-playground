import { useEffect, useLayoutEffect, useRef } from 'react'
import type { Algorithm, Obstacle, PlanEvent, Point } from '../engine/types'
import type { TimedPlan } from '../engine/planner'
import { ALGO_BY_ID, COLORS, ROBOT_RADIUS, WORLD_H, WORLD_W, easeInOutCubic, easeOutBack, easeOutExpo } from '../lib/theme'
import { clamp, cumulative, dist, normRect, pointInObstacle, sampleAt } from '../lib/geometry'
import { drawChecker, drawField, drawObstacle, fitCanvas, label, makeLayer, rgba, strokePartial, type Surface } from '../lib/draw'

export type Tool = 'rect' | 'circle' | 'move' | 'erase'
export type RunPhase = 'explore' | 'path' | 'drive' | 'arrived' | 'failed'

export interface RunView {
  key: string
  algo: Algorithm
  result: TimedPlan
}

export interface RunProgress {
  key: string
  algo: Algorithm
  phase: RunPhase
  frac: number
}

interface Props {
  obstacles: Obstacle[]
  start: Point
  goal: Point
  tool: Tool
  gridSize: number
  runs: RunView[]
  race: boolean
  skipToken: number
  reduced: boolean
  onObstacles: (next: Obstacle[]) => void
  onStart: (p: Point) => void
  onGoal: (p: Point) => void
  onEdit: () => void
  onReject: (msg: string) => void
  onProgress: (p: RunProgress) => void
}

interface Edge { x: number; y: number; px: number; py: number }

interface RunAnim {
  view: RunView
  color: string
  layer: HTMLCanvasElement
  lctx: CanvasRenderingContext2D
  played: number
  t0: number
  rate: number
  parents: Map<string, Edge>
  flashes: { e: Edge; t: number }[]
  phase: RunPhase
  phaseT: number
  cum: number[]
  total: number
  removing: number | null
  lastReport: number
  head: Point | null
}

const DRIVE_SPEED = 330 // world units / second, identical for every robot so the shortest path wins
const SINGLE_REPLAY_MS = 2400
const RACE_REPLAY_MS = 3600

type Drag =
  | { kind: 'start' | 'goal' }
  | { kind: 'rect'; x0: number; y0: number; x1: number; y1: number }
  | { kind: 'circle'; x0: number; y0: number; r: number }
  | { kind: 'move'; index: number; dx: number; dy: number; orig: Obstacle }

export function RobotCanvas(props: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const propsRef = useRef(props)
  // Keep the latest props for the animation loop and pointer handlers, synced after each commit.
  useLayoutEffect(() => {
    propsRef.current = props
  })

  useEffect(() => {
    const canvas = canvasRef.current!
    const ctx = canvas.getContext('2d')!
    let surface: Surface = fitCanvas(canvas, WORLD_W, WORLD_H)
    let raf = 0
    const mountT = performance.now()
    let last = mountT

    const births = new WeakMap<Obstacle, number>()
    const silent = new WeakSet<Obstacle>()
    let prevObstacles: Obstacle[] = []
    const ghosts: { o: Obstacle; t: number }[] = []
    const runs = new Map<string, RunAnim>()
    let lastSkip = propsRef.current.skipToken
    let armatureAlpha = 0

    // Markers follow their targets with a spring so drags and preset swaps feel physical.
    const marker = {
      start: { ...propsRef.current.start, vx: 0, vy: 0 },
      goal: { ...propsRef.current.goal, vx: 0, vy: 0 },
    }

    let drag: Drag | null = null
    let activePointer: number | null = null
    let hover: Point | null = null
    let rejectPulse = 0

    const px = () => 1 / (surface.scale / surface.dpr) // one CSS pixel in world units

    // ---------- run layers ----------
    const keyOf = (x: number, y: number) => `${x.toFixed(2)},${y.toFixed(2)}`

    function paintEdge(r: RunAnim, e: Edge, alpha: number) {
      const c = r.lctx
      c.strokeStyle = rgba(r.color, alpha)
      c.beginPath()
      c.moveTo(e.px, e.py)
      c.lineTo(e.x, e.y)
      c.stroke()
    }

    function prepLayerCtx(r: RunAnim) {
      const c = r.lctx
      c.setTransform(surface.scale, 0, 0, surface.scale, 0, 0)
      c.lineWidth = 1.1 * px()
      c.lineCap = 'round'
    }

    function treeAlpha() {
      return propsRef.current.race ? 0.42 : 0.62
    }

    function redrawTree(r: RunAnim) {
      r.lctx.setTransform(1, 0, 0, 1, 0, 0)
      r.lctx.clearRect(0, 0, r.layer.width, r.layer.height)
      prepLayerCtx(r)
      if (r.view.algo === 'astar') {
        applyAstar(r, 0, r.played)
        return
      }
      const a = treeAlpha()
      for (const e of r.parents.values()) paintEdge(r, e, a)
    }

    function applyAstar(r: RunAnim, from: number, to: number) {
      const g = propsRef.current.gridSize
      const ev = r.view.result.events
      const c = r.lctx
      for (let i = from; i < to; i++) {
        const e = ev[i]
        if (e.k === 'expand') {
          c.fillStyle = rgba(r.color, propsRef.current.race ? 0.13 : 0.2)
          c.fillRect(e.x - g / 2 + 0.6, e.y - g / 2 + 0.6, g - 1.2, g - 1.2)
        } else if (e.k === 'frontier') {
          c.strokeStyle = rgba(r.color, 0.4)
          c.strokeRect(e.x - g / 2 + 1.5, e.y - g / 2 + 1.5, g - 3, g - 3)
        }
      }
    }

    function applyEvents(r: RunAnim, to: number, now: number) {
      const ev = r.view.result.events
      const from = r.played
      if (to <= from) return
      prepLayerCtx(r)
      if (r.view.algo === 'astar') {
        applyAstar(r, from, to)
      } else {
        let rewired = false
        const a = treeAlpha()
        for (let i = from; i < to; i++) {
          const e = ev[i] as PlanEvent
          if (e.k === 'expand') {
            const edge = { x: e.x, y: e.y, px: e.px, py: e.py }
            r.parents.set(keyOf(e.x, e.y), edge)
            if (!rewired) paintEdge(r, edge, a)
          } else if (e.k === 'rewire') {
            const edge = { x: e.x, y: e.y, px: e.px, py: e.py }
            r.parents.set(keyOf(e.x, e.y), edge)
            rewired = true
            if (r.flashes.length < 60) r.flashes.push({ e: edge, t: now })
          }
        }
        r.played = to
        if (rewired) redrawTree(r)
      }
      r.played = to
      const lastEv = ev[to - 1]
      if (lastEv) r.head = { x: lastEv.x, y: lastEv.y }
    }

    function createRun(view: RunView, now: number, rate: number): RunAnim {
      const [layer, lctx] = makeLayer(canvas)
      const path = view.result.path
      const cum = cumulative(path)
      return {
        view,
        color: ALGO_BY_ID[view.algo].color,
        layer,
        lctx,
        played: 0,
        t0: now + 120,
        rate,
        parents: new Map(),
        flashes: [],
        phase: 'explore',
        phaseT: now,
        cum,
        total: cum[cum.length - 1] ?? 0,
        removing: null,
        lastReport: 0,
        head: null,
      }
    }

    function syncRuns(now: number) {
      const views = propsRef.current.runs
      const keys = new Set(views.map((v) => v.key))
      for (const [k, r] of runs) {
        if (!keys.has(k) && r.removing === null) r.removing = now
      }
      const fresh = views.filter((v) => !runs.has(v.key))
      if (fresh.length) {
        const maxN = Math.max(...fresh.map((v) => v.result.events.length), 1)
        const span = propsRef.current.race ? RACE_REPLAY_MS : SINGLE_REPLAY_MS
        const rate = Math.max(maxN / span, 0.35)
        for (const v of fresh) runs.set(v.key, createRun(v, now, rate))
      }
    }

    function report(r: RunAnim, now: number, force = false) {
      if (!force && now - r.lastReport < 90) return
      r.lastReport = now
      const n = r.view.result.events.length
      let frac = 1
      if (r.phase === 'explore') frac = n ? r.played / n : 1
      else if (r.phase === 'path') frac = clamp((now - r.phaseT) / pathDuration(r), 0, 1)
      else if (r.phase === 'drive') frac = r.total ? clamp(((now - r.phaseT) / 1000) * DRIVE_SPEED / r.total, 0, 1) : 1
      propsRef.current.onProgress({ key: r.view.key, algo: r.view.algo, phase: r.phase, frac })
    }

    function setPhase(r: RunAnim, phase: RunPhase, now: number) {
      r.phase = phase
      r.phaseT = now
      report(r, now, true)
    }

    const pathDuration = (r: RunAnim) => clamp(r.total * 0.9, 520, 1100)

    function finishInstantly(r: RunAnim, now: number) {
      const n = r.view.result.events.length
      if (r.played < n) applyEvents(r, n, now)
      r.flashes = []
      if (!r.view.result.found) setPhase(r, 'failed', now)
      else setPhase(r, 'arrived', now)
    }

    function stepRun(r: RunAnim, now: number) {
      if (r.phase === 'arrived' || r.phase === 'failed') return
      const reduced = propsRef.current.reduced
      const n = r.view.result.events.length
      if (reduced) {
        finishInstantly(r, now)
        return
      }
      if (r.phase === 'explore') {
        const target = Math.min(n, Math.max(0, Math.floor((now - r.t0) * r.rate)))
        applyEvents(r, target, now)
        report(r, now)
        if (r.played >= n && now >= r.t0) {
          r.head = null
          setPhase(r, r.view.result.found ? 'path' : 'failed', now)
        }
      } else if (r.phase === 'path') {
        report(r, now)
        if (now - r.phaseT >= pathDuration(r) + 140) setPhase(r, 'drive', now)
      } else if (r.phase === 'drive') {
        report(r, now)
        const d = ((now - r.phaseT) / 1000) * DRIVE_SPEED
        if (d >= r.total) setPhase(r, 'arrived', now)
      }
    }

    // ---------- drawing ----------
    function drawRobot(x: number, y: number, angle: number, color: string, alpha: number) {
      const R = ROBOT_RADIUS
      ctx.save()
      ctx.globalAlpha = alpha
      ctx.translate(x, y)
      ctx.rotate(angle)
      ctx.fillStyle = '#0B0C0E'
      ctx.beginPath()
      ctx.roundRect(-R * 0.55, -R - 3, R * 1.1, 5, 2)
      ctx.roundRect(-R * 0.55, R - 2, R * 1.1, 5, 2)
      ctx.fill()
      ctx.beginPath()
      ctx.arc(0, 0, R, 0, Math.PI * 2)
      ctx.fillStyle = '#16191D'
      ctx.fill()
      ctx.lineWidth = 2.6 * px()
      ctx.strokeStyle = color
      ctx.stroke()
      ctx.beginPath()
      ctx.moveTo(R * 0.85, 0)
      ctx.lineTo(R * 0.05, -R * 0.45)
      ctx.lineTo(R * 0.05, R * 0.45)
      ctx.closePath()
      ctx.fillStyle = color
      ctx.fill()
      ctx.restore()
    }

    function drawArmature(g: number, a: number) {
      if (a <= 0.01) return
      ctx.save()
      ctx.strokeStyle = `rgba(255,122,69,${0.07 * a})`
      ctx.lineWidth = px()
      ctx.beginPath()
      for (let x = 0; x <= WORLD_W; x += g) {
        ctx.moveTo(x, 0)
        ctx.lineTo(x, WORLD_H)
      }
      for (let y = 0; y <= WORLD_H; y += g) {
        ctx.moveTo(0, y)
        ctx.lineTo(WORLD_W, y)
      }
      ctx.stroke()
      ctx.restore()
    }

    function overlapsMarkers(o: Obstacle) {
      const { start, goal } = propsRef.current
      return pointInObstacle(o, start, ROBOT_RADIUS) || pointInObstacle(o, goal, ROBOT_RADIUS)
    }

    function draftObstacle(): Obstacle | null {
      if (!drag) return null
      if (drag.kind === 'rect') return normRect(drag.x0, drag.y0, drag.x1, drag.y1)
      if (drag.kind === 'circle') return { kind: 'circle', x: drag.x0, y: drag.y0, r: drag.r }
      return null
    }

    function frame(now: number) {
      raf = requestAnimationFrame(frame)
      const dt = Math.min(1 / 30, (now - last) / 1000)
      last = now
      const P = propsRef.current
      const reduced = P.reduced

      // obstacle births and deaths
      let batch = 0
      for (const o of P.obstacles) {
        if (!births.has(o)) births.set(o, reduced ? -1e9 : now + Math.min(batch++ * 16, 520))
      }
      if (prevObstacles !== P.obstacles) {
        const cur = new Set(P.obstacles)
        for (const o of prevObstacles) {
          if (!cur.has(o) && !silent.has(o) && !reduced) ghosts.push({ o, t: now })
        }
        prevObstacles = P.obstacles
      }

      if (P.skipToken !== lastSkip) {
        lastSkip = P.skipToken
        for (const r of runs.values()) if (r.removing === null) finishInstantly(r, now)
      }
      syncRuns(now)
      for (const r of runs.values()) if (r.removing === null) stepRun(r, now)

      // springs for markers
      const stiff = 520
      const damp = 2 * Math.sqrt(stiff) * 0.92
      for (const name of ['start', 'goal'] as const) {
        const m = marker[name]
        const t = P[name]
        if (reduced || (drag && drag.kind === name)) {
          m.x = t.x
          m.y = t.y
          m.vx = m.vy = 0
          continue
        }
        m.vx += ((t.x - m.x) * stiff - m.vx * damp) * dt
        m.vy += ((t.y - m.y) * stiff - m.vy * damp) * dt
        m.x += m.vx * dt
        m.y += m.vy * dt
      }

      ctx.setTransform(1, 0, 0, 1, 0, 0)
      ctx.clearRect(0, 0, canvas.width, canvas.height)
      const reveal = reduced ? 1 : easeOutExpo(clamp((now - mountT) / 1100, 0, 1))
      ctx.setTransform(surface.scale, 0, 0, surface.scale, 0, 0)
      drawField(ctx, WORLD_W, WORLD_H, 40, 1.1 * px(), reveal)

      // A* armature: the grid is shown as a real structure only while A* is in play
      const wantArmature = [...runs.values()].some((r) => r.view.algo === 'astar' && r.removing === null)
      armatureAlpha += ((wantArmature ? 1 : 0) - armatureAlpha) * Math.min(1, dt * 6)
      drawArmature(P.gridSize, armatureAlpha)

      // obstacles
      const unit = px()
      for (let i = ghosts.length - 1; i >= 0; i--) {
        const g = ghosts[i]
        const t = (now - g.t) / 170
        if (t >= 1) {
          ghosts.splice(i, 1)
          continue
        }
        drawObstacle(ctx, g.o, { scale: 1 - 0.1 * t, alpha: 1 - t, px: unit, danger: 1 - t })
      }
      const draggingIdx = drag && drag.kind === 'move' ? drag.index : -1
      P.obstacles.forEach((o, i) => {
        const b = births.get(o) ?? 0
        const t = clamp((now - b) / 340, 0, 1)
        if (now < b) return
        const hoverErase = P.tool === 'erase' && hover && pointInObstacle(o, hover) ? 1 : 0
        const lift = i === draggingIdx ? 1.02 : 1
        drawObstacle(ctx, o, {
          scale: (0.84 + 0.16 * easeOutBack(t)) * lift,
          alpha: easeOutExpo(t),
          px: unit,
          danger: hoverErase,
        })
      })

      // run layers
      ctx.setTransform(1, 0, 0, 1, 0, 0)
      for (const [k, r] of runs) {
        let a = 1
        if (r.removing !== null) {
          a = 1 - (now - r.removing) / 220
          if (a <= 0 || reduced) {
            runs.delete(k)
            continue
          }
        } else if (r.phase === 'failed') {
          a = 0.55
        }
        ctx.globalAlpha = a
        ctx.drawImage(r.layer, 0, 0)
      }
      ctx.globalAlpha = 1
      ctx.setTransform(surface.scale, 0, 0, surface.scale, 0, 0)

      for (const r of runs.values()) {
        if (r.removing !== null) continue
        // rewire flashes (RRT*)
        for (let i = r.flashes.length - 1; i >= 0; i--) {
          const f = r.flashes[i]
          const t = (now - f.t) / 420
          if (t >= 1) {
            r.flashes.splice(i, 1)
            continue
          }
          ctx.strokeStyle = `rgba(247,248,250,${0.85 * (1 - t)})`
          ctx.lineWidth = 2 * unit
          ctx.beginPath()
          ctx.moveTo(f.e.px, f.e.py)
          ctx.lineTo(f.e.x, f.e.y)
          ctx.stroke()
        }
        if (r.phase === 'explore' && r.head) {
          const pulse = 0.5 + 0.5 * Math.sin(now / 90)
          ctx.fillStyle = rgba(r.color, 0.25)
          ctx.beginPath()
          ctx.arc(r.head.x, r.head.y, (7 + pulse * 3) * unit, 0, Math.PI * 2)
          ctx.fill()
          ctx.fillStyle = COLORS.white
          ctx.beginPath()
          ctx.arc(r.head.x, r.head.y, 2.6 * unit, 0, Math.PI * 2)
          ctx.fill()
        }
      }

      // paths
      ctx.lineCap = 'round'
      ctx.lineJoin = 'round'
      for (const r of runs.values()) {
        if (r.removing !== null || !r.view.result.found) continue
        if (r.phase === 'explore') continue
        const path = r.view.result.path
        let upto = r.total
        if (r.phase === 'path') upto = r.total * easeInOutCubic(clamp((now - r.phaseT) / pathDuration(r), 0, 1))
        ctx.strokeStyle = 'rgba(11,12,14,0.85)'
        ctx.lineWidth = 6.5 * unit
        strokePartial(ctx, path, r.cum, upto)
        ctx.strokeStyle = r.color
        ctx.lineWidth = 3 * unit
        strokePartial(ctx, path, r.cum, upto)
        if (r.phase === 'path' && upto < r.total) {
          const tip = sampleAt(path, r.cum, upto)
          ctx.fillStyle = COLORS.white
          ctx.beginPath()
          ctx.arc(tip.x, tip.y, 3.4 * unit, 0, Math.PI * 2)
          ctx.fill()
        }
      }

      // markers
      const s = marker.start
      const g = marker.goal
      const anyFailed = [...runs.values()].some((r) => r.phase === 'failed' && r.removing === null)
      ctx.setLineDash([3 * unit, 4 * unit])
      ctx.strokeStyle = 'rgba(247,248,250,0.5)'
      ctx.lineWidth = 1.2 * unit
      ctx.beginPath()
      ctx.arc(s.x, s.y, ROBOT_RADIUS + 6, 0, Math.PI * 2)
      ctx.stroke()
      ctx.setLineDash([])
      ctx.fillStyle = COLORS.white
      ctx.beginPath()
      ctx.arc(s.x, s.y, 7, 0, Math.PI * 2)
      ctx.fill()
      ctx.fillStyle = '#0B0C0E'
      ctx.beginPath()
      ctx.arc(s.x, s.y, 2.6, 0, Math.PI * 2)
      ctx.fill()
      label(ctx, 'Start', s.x, s.y - ROBOT_RADIUS - 10, unit, COLORS.ink)

      drawChecker(ctx, g.x, g.y, 10, unit)
      if (anyFailed) {
        ctx.strokeStyle = rgba(COLORS.danger, 0.9)
        ctx.lineWidth = 2.2 * unit
        ctx.beginPath()
        ctx.arc(g.x, g.y, 17 + Math.sin(now / 180) * 1.5, 0, Math.PI * 2)
        ctx.stroke()
      }
      label(ctx, 'Goal', g.x, g.y - 18, unit, COLORS.ink)

      // robots: parked at start while exploring, then driving
      const active = [...runs.values()].filter((r) => r.removing === null)
      active.forEach((r, i) => {
        const path = r.view.result.path
        if (!r.view.result.found || path.length < 2) return
        if (r.phase === 'explore' || r.phase === 'path') {
          if (r.phase === 'path') {
            const t = easeOutExpo(clamp((now - r.phaseT) / 400, 0, 1))
            const first = sampleAt(path, r.cum, 0)
            drawRobot(first.x, first.y, first.angle, r.color, t)
          }
          return
        }
        const d = r.phase === 'arrived' ? r.total : ((now - r.phaseT) / 1000) * DRIVE_SPEED
        const p = sampleAt(path, r.cum, d)
        const offset = active.length > 1 && r.phase === 'arrived' ? (i - 1) * 4 : 0
        drawRobot(p.x + offset, p.y + offset, p.angle, r.color, 1)
      })

      // draft obstacle while drawing
      const draft = draftObstacle()
      if (draft) {
        const bad = overlapsMarkers(draft)
        drawObstacle(ctx, draft, { scale: 1, alpha: 0.85, px: unit, danger: bad ? 0.9 : 0 })
        ctx.setLineDash([5 * unit, 4 * unit])
        ctx.strokeStyle = bad ? COLORS.danger : COLORS.white
        ctx.lineWidth = 1.4 * unit
        if (draft.kind === 'rect') ctx.strokeRect(draft.x, draft.y, draft.w, draft.h)
        else {
          ctx.beginPath()
          ctx.arc(draft.x, draft.y, draft.r, 0, Math.PI * 2)
          ctx.stroke()
        }
        ctx.setLineDash([])
      }

      if (rejectPulse > 0) {
        rejectPulse = Math.max(0, rejectPulse - dt * 3)
        ctx.strokeStyle = rgba(COLORS.danger, rejectPulse)
        ctx.lineWidth = 3 * unit
        ctx.strokeRect(1.5 * unit, 1.5 * unit, WORLD_W - 3 * unit, WORLD_H - 3 * unit)
      }
    }

    raf = requestAnimationFrame(frame)

    // ---------- input ----------
    const toWorld = (e: PointerEvent): Point => {
      const rect = canvas.getBoundingClientRect()
      return {
        x: clamp(((e.clientX - rect.left) / rect.width) * WORLD_W, 0, WORLD_W),
        y: clamp(((e.clientY - rect.top) / rect.height) * WORLD_H, 0, WORLD_H),
      }
    }

    function onDown(e: PointerEvent) {
      if (activePointer !== null) return // ignore extra fingers mid-gesture
      const P = propsRef.current
      const p = toWorld(e)
      const slop = (e.pointerType === 'touch' ? 34 : 20) * px()
      const ds = dist(p, marker.start)
      const dg = dist(p, marker.goal)
      if (Math.min(ds, dg) < Math.max(slop, 22)) {
        drag = { kind: ds <= dg ? 'start' : 'goal' }
      } else if (P.tool === 'rect') {
        drag = { kind: 'rect', x0: p.x, y0: p.y, x1: p.x, y1: p.y }
      } else if (P.tool === 'circle') {
        drag = { kind: 'circle', x0: p.x, y0: p.y, r: 0 }
      } else if (P.tool === 'move' || P.tool === 'erase') {
        let idx = -1
        for (let i = P.obstacles.length - 1; i >= 0; i--) {
          if (pointInObstacle(P.obstacles[i], p, e.pointerType === 'touch' ? 8 : 2)) {
            idx = i
            break
          }
        }
        if (idx < 0) return
        if (P.tool === 'erase') {
          P.onEdit()
          P.onObstacles(P.obstacles.filter((_, i) => i !== idx))
          return
        }
        const o = P.obstacles[idx]
        drag = { kind: 'move', index: idx, dx: p.x - o.x, dy: p.y - o.y, orig: o }
      } else return
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
      updateCursor(p)
      if (e.pointerId !== activePointer || !drag) return
      const P = propsRef.current
      if (drag.kind === 'start' || drag.kind === 'goal') {
        P.onEdit()
        const q = { x: clamp(p.x, ROBOT_RADIUS, WORLD_W - ROBOT_RADIUS), y: clamp(p.y, ROBOT_RADIUS, WORLD_H - ROBOT_RADIUS) }
        if (drag.kind === 'start') P.onStart(q)
        else P.onGoal(q)
      } else if (drag.kind === 'rect') {
        drag.x1 = p.x
        drag.y1 = p.y
      } else if (drag.kind === 'circle') {
        drag.r = dist(p, { x: drag.x0, y: drag.y0 })
      } else if (drag.kind === 'move') {
        const d = drag
        const o = P.obstacles[d.index]
        if (!o) return
        const next: Obstacle = { ...o, x: p.x - d.dx, y: p.y - d.dy }
        births.set(next, -1e9)
        silent.add(o)
        P.onEdit()
        P.onObstacles(P.obstacles.map((q, i) => (i === d.index ? next : q)))
      }
    }

    function onUp(e: PointerEvent) {
      if (e.pointerId !== activePointer) return
      activePointer = null
      const P = propsRef.current
      if (drag && (drag.kind === 'rect' || drag.kind === 'circle')) {
        let o = draftObstacle()!
        if (o.kind === 'rect' && o.w < 8 && o.h < 8) o = { kind: 'rect', x: o.x - 30, y: o.y - 30, w: 60, h: 60 }
        else if (o.kind === 'rect') o = { ...o, w: Math.max(o.w, 6), h: Math.max(o.h, 6) }
        if (o.kind === 'circle' && o.r < 6) o = { ...o, r: 30 }
        if (overlapsMarkers(o)) {
          rejectPulse = 1
          P.onReject('Keep walls clear of the start and the goal.')
        } else {
          P.onEdit()
          P.onObstacles([...P.obstacles, o])
        }
      } else if (drag && drag.kind === 'move') {
        const d = drag
        const o = P.obstacles[d.index]
        if (o && overlapsMarkers(o)) {
          rejectPulse = 1
          births.set(d.orig, -1e9)
          silent.add(o)
          P.onObstacles(P.obstacles.map((q, i) => (i === d.index ? d.orig : q)))
          P.onReject('That spot covers the start or the goal, so the wall went back.')
        }
      }
      drag = null
    }

    function updateCursor(p: Point) {
      const P = propsRef.current
      if (drag) return
      const near = Math.min(dist(p, marker.start), dist(p, marker.goal)) < 22
      let c = 'crosshair'
      if (near) c = 'grab'
      else if (P.tool === 'move') c = P.obstacles.some((o) => pointInObstacle(o, p, 2)) ? 'move' : 'default'
      else if (P.tool === 'erase') c = P.obstacles.some((o) => pointInObstacle(o, p, 2)) ? 'pointer' : 'default'
      canvas.style.cursor = c
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
      for (const r of runs.values()) {
        r.layer.width = canvas.width
        r.layer.height = canvas.height
        r.lctx = r.layer.getContext('2d')!
        redrawTree(r)
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
        aria-label="Planning field. Drag on it to draw walls, drag the start and the goal to move them."
      />
    </div>
  )
}
