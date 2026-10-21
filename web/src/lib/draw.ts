import type { Obstacle, Point } from '../engine/types'
import { COLORS } from './theme'

const hexCache = new Map<string, [number, number, number]>()
export function rgba(hex: string, a: number): string {
  let c = hexCache.get(hex)
  if (!c) {
    const h = hex.replace('#', '')
    c = [parseInt(h.slice(0, 2), 16), parseInt(h.slice(2, 4), 16), parseInt(h.slice(4, 6), 16)]
    hexCache.set(hex, c)
  }
  return `rgba(${c[0]},${c[1]},${c[2]},${a})`
}

export function mixHex(a: string, b: string, t: number): string {
  rgba(a, 1)
  rgba(b, 1)
  const ca = hexCache.get(a)!
  const cb = hexCache.get(b)!
  const m = (i: number) => Math.round(ca[i] + (cb[i] - ca[i]) * t)
  return `rgb(${m(0)},${m(1)},${m(2)})`
}

export interface Surface {
  cssW: number
  cssH: number
  dpr: number
  /** world units -> device pixels */
  scale: number
}

export function fitCanvas(canvas: HTMLCanvasElement, worldW: number, worldH: number): Surface {
  const parent = canvas.parentElement!
  const cssW = Math.max(1, parent.clientWidth)
  const cssH = (cssW * worldH) / worldW
  const dpr = Math.min(2.5, window.devicePixelRatio || 1)
  const w = Math.round(cssW * dpr)
  const h = Math.round(cssH * dpr)
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w
    canvas.height = h
  }
  canvas.style.height = `${cssH}px`
  return { cssW, cssH, dpr, scale: w / worldW }
}

export function makeLayer(like: HTMLCanvasElement): [HTMLCanvasElement, CanvasRenderingContext2D] {
  const c = document.createElement('canvas')
  c.width = like.width
  c.height = like.height
  return [c, c.getContext('2d')!]
}

/** Survey marks every `step` units; the sweep reveals them left to right on load. */
export function drawField(ctx: CanvasRenderingContext2D, w: number, h: number, step: number, dotR: number, reveal: number) {
  ctx.fillStyle = COLORS.field
  ctx.fillRect(0, 0, w, h)
  ctx.fillStyle = 'rgba(236,238,241,0.13)'
  for (let x = step; x < w; x += step) {
    const a = Math.min(1, Math.max(0, reveal * 1.6 - x / w))
    if (a <= 0) continue
    ctx.globalAlpha = a
    for (let y = step; y < h; y += step) {
      ctx.fillRect(x - dotR, y - dotR, dotR * 2, dotR * 2)
    }
  }
  ctx.globalAlpha = 1
}

export function obstaclePath(ctx: CanvasRenderingContext2D, o: Obstacle, radius: number) {
  ctx.beginPath()
  if (o.kind === 'rect') {
    const r = Math.min(radius, o.w / 2, o.h / 2)
    ctx.roundRect(o.x, o.y, o.w, o.h, r)
  } else {
    ctx.arc(o.x, o.y, o.r, 0, Math.PI * 2)
  }
}

export function drawObstacle(
  ctx: CanvasRenderingContext2D,
  o: Obstacle,
  opts: { scale: number; alpha: number; px: number; danger?: number },
) {
  const cx = o.kind === 'rect' ? o.x + o.w / 2 : o.x
  const cy = o.kind === 'rect' ? o.y + o.h / 2 : o.y
  ctx.save()
  ctx.globalAlpha = opts.alpha
  ctx.translate(cx, cy)
  ctx.scale(opts.scale, opts.scale)
  ctx.translate(-cx, -cy)
  obstaclePath(ctx, o, 4 * opts.px)
  ctx.fillStyle = COLORS.wall
  ctx.fill()
  ctx.lineWidth = 1.25 * opts.px
  ctx.strokeStyle = COLORS.wallEdge
  ctx.stroke()
  if (opts.danger && opts.danger > 0) {
    ctx.fillStyle = rgba(COLORS.danger, 0.22 * opts.danger)
    ctx.fill()
    ctx.strokeStyle = rgba(COLORS.danger, opts.danger)
    ctx.lineWidth = 2 * opts.px
    ctx.stroke()
  }
  ctx.restore()
}

/** Chequered flag disc for the goal. */
export function drawChecker(ctx: CanvasRenderingContext2D, x: number, y: number, r: number, px: number) {
  ctx.save()
  ctx.beginPath()
  ctx.arc(x, y, r, 0, Math.PI * 2)
  ctx.clip()
  const n = 4
  const s = (r * 2) / n
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      ctx.fillStyle = (i + j) % 2 === 0 ? COLORS.white : '#0B0C0E'
      ctx.fillRect(x - r + i * s, y - r + j * s, s, s)
    }
  }
  ctx.restore()
  ctx.beginPath()
  ctx.arc(x, y, r, 0, Math.PI * 2)
  ctx.lineWidth = 2 * px
  ctx.strokeStyle = COLORS.white
  ctx.stroke()
}

/** Stroke a polyline up to arc length `upto`. */
export function strokePartial(ctx: CanvasRenderingContext2D, path: Point[], cum: number[], upto: number) {
  if (path.length < 2) return
  ctx.beginPath()
  ctx.moveTo(path[0].x, path[0].y)
  for (let i = 1; i < path.length; i++) {
    if (cum[i] <= upto) {
      ctx.lineTo(path[i].x, path[i].y)
    } else {
      const seg = cum[i] - cum[i - 1] || 1
      const u = (upto - cum[i - 1]) / seg
      ctx.lineTo(path[i - 1].x + (path[i].x - path[i - 1].x) * u, path[i - 1].y + (path[i].y - path[i - 1].y) * u)
      break
    }
  }
  ctx.stroke()
}

export function label(ctx: CanvasRenderingContext2D, text: string, x: number, y: number, px: number, color: string) {
  ctx.save()
  ctx.font = `600 ${12 * px}px "Barlow Condensed", "Arial Narrow", sans-serif`
  ctx.textAlign = 'center'
  ctx.textBaseline = 'bottom'
  ctx.fillStyle = color
  ctx.fillText(text.toUpperCase(), x, y)
  ctx.restore()
}
