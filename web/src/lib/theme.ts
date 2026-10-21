import type { Algorithm } from '../engine/types'

export const WORLD_W = 1000
export const WORLD_H = 640
export const ROBOT_RADIUS = 12

export interface AlgoMeta {
  id: Algorithm
  name: string
  blurb: string
  color: string
}

/** Each planner wears one livery colour everywhere: canvas, tower, benchmark. */
export const ALGOS: AlgoMeta[] = [
  { id: 'astar', name: 'A*', blurb: 'Grid search, optimal on the grid', color: '#FF7A45' },
  { id: 'rrt', name: 'RRT', blurb: 'Random tree, fast and rough', color: '#34D1C4' },
  { id: 'rrtstar', name: 'RRT*', blurb: 'Random tree that rewires toward optimal', color: '#E8E15A' },
]

export const ALGO_BY_ID: Record<Algorithm, AlgoMeta> = {
  astar: ALGOS[0],
  rrt: ALGOS[1],
  rrtstar: ALGOS[2],
}

export const COLORS = {
  ground: '#121417',
  field: '#171A1E',
  fieldLine: 'rgba(236, 238, 241, 0.045)',
  wall: '#2B3038',
  wallEdge: '#3B424D',
  ink: '#ECEEF1',
  inkSoft: '#9AA3AE',
  best: '#B072FF',
  danger: '#FF4D5E',
  white: '#F7F8FA',
}

/** Motion tokens (Emil: strong custom curves, UI under 300ms). */
export const EASE_OUT = [0.23, 1, 0.32, 1] as const
export const EASE_IN_OUT = [0.77, 0, 0.175, 1] as const
export const EASE_DRAWER = [0.32, 0.72, 0, 1] as const

export function easeOutExpo(t: number): number {
  return t >= 1 ? 1 : 1 - Math.pow(2, -10 * t)
}
export function easeOutBack(t: number): number {
  const c1 = 1.4
  const c3 = c1 + 1
  return 1 + c3 * Math.pow(t - 1, 3) + c1 * Math.pow(t - 1, 2)
}
export function easeInOutCubic(t: number): number {
  return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2
}
