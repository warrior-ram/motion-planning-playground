import type { Obstacle, Point } from '../engine/types'

export const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v))
export const dist = (a: Point, b: Point) => Math.hypot(a.x - b.x, a.y - b.y)

export function pointInObstacle(o: Obstacle, p: Point, pad = 0): boolean {
  if (o.kind === 'rect') {
    return p.x >= o.x - pad && p.x <= o.x + o.w + pad && p.y >= o.y - pad && p.y <= o.y + o.h + pad
  }
  return Math.hypot(p.x - o.x, p.y - o.y) <= o.r + pad
}

export function obstacleCenter(o: Obstacle): Point {
  return o.kind === 'rect' ? { x: o.x + o.w / 2, y: o.y + o.h / 2 } : { x: o.x, y: o.y }
}

/** Cumulative arc length along a polyline. */
export function cumulative(path: Point[]): number[] {
  const c = [0]
  for (let i = 1; i < path.length; i++) c.push(c[i - 1] + dist(path[i - 1], path[i]))
  return c
}

/** Position and heading at arc length d. */
export function sampleAt(path: Point[], cum: number[], d: number): { x: number; y: number; angle: number } {
  if (path.length === 0) return { x: 0, y: 0, angle: 0 }
  if (path.length === 1) return { ...path[0], angle: 0 }
  const total = cum[cum.length - 1]
  const t = clamp(d, 0, total)
  let i = 1
  while (i < cum.length - 1 && cum[i] < t) i++
  const a = path[i - 1]
  const b = path[i]
  const seg = cum[i] - cum[i - 1] || 1
  const u = (t - cum[i - 1]) / seg
  return { x: a.x + (b.x - a.x) * u, y: a.y + (b.y - a.y) * u, angle: Math.atan2(b.y - a.y, b.x - a.x) }
}

/**
 * Mirror of the contract's fk (x += L cos θ, relative joint angles).
 * Used only to project thousands of joint-space tree events to the end effector
 * for display without a JSON round-trip per event; the arm itself is drawn from
 * the WASM fk.
 */
export function fkLocal(base: Point, links: number[], angles: number[]): Point[] {
  const out: Point[] = [{ ...base }]
  let x = base.x
  let y = base.y
  let a = 0
  for (let i = 0; i < links.length; i++) {
    a += angles[i] ?? 0
    x += links[i] * Math.cos(a)
    y += links[i] * Math.sin(a)
    out.push({ x, y })
  }
  return out
}

/** Smallest signed difference b - a, wrapped to (-π, π]. */
export function angleDelta(a: number, b: number): number {
  let d = (b - a) % (Math.PI * 2)
  if (d > Math.PI) d -= Math.PI * 2
  if (d <= -Math.PI) d += Math.PI * 2
  return d
}

export function normRect(x0: number, y0: number, x1: number, y1: number): Obstacle {
  return { kind: 'rect', x: Math.min(x0, x1), y: Math.min(y0, y1), w: Math.abs(x1 - x0), h: Math.abs(y1 - y0) }
}

export function fmt(n: number, decimals = 0): string {
  return n.toLocaleString('en-US', { minimumFractionDigits: decimals, maximumFractionDigits: decimals })
}
