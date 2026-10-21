// Types mirror CONTRACT.md exactly (camelCase JSON).

export interface Point { x: number; y: number }

export type Obstacle =
  | { kind: 'rect'; x: number; y: number; w: number; h: number }
  | { kind: 'circle'; x: number; y: number; r: number }

export type Algorithm = 'astar' | 'rrt' | 'rrtstar'
export type ArmAlgorithm = 'rrt' | 'rrtstar'

export interface PlanParams {
  gridSize?: number
  maxIterations?: number
  stepSize?: number
  goalBias?: number
  goalTolerance?: number
  rewireRadius?: number
}

export interface PlanRequest {
  algorithm: Algorithm
  width: number
  height: number
  obstacles: Obstacle[]
  start: Point
  goal: Point
  robotRadius: number
  seed: number
  params?: PlanParams
}

export type PlanEvent =
  | { k: 'expand'; x: number; y: number; px: number; py: number }
  | { k: 'rewire'; x: number; y: number; px: number; py: number; opx: number; opy: number }
  | { k: 'frontier'; x: number; y: number }

export interface PlanResponse {
  found: boolean
  path: Point[]
  pathLength: number
  nodesExplored: number
  iterations: number
  events: PlanEvent[]
}

export interface BenchMap {
  id: string
  name: string
  width: number
  height: number
  start: Point
  goal: Point
  obstacles: Obstacle[]
}

export interface ArmBase { base: Point; links: number[] }

export interface FkResponse { joints: Point[] }

export interface IkRequest extends ArmBase {
  angles: number[]
  target: Point
  limits?: [number, number][]
  maxIterations?: number
  tolerance?: number
}

export interface IkResponse {
  angles: number[]
  joints: Point[]
  reached: boolean
  error: number
  iterations: number
}

export interface PlanArmRequest extends ArmBase {
  obstacles: Obstacle[]
  linkThickness: number
  start: number[]
  goal: number[]
  algorithm: ArmAlgorithm
  seed: number
  params?: {
    maxIterations?: number
    stepSize?: number
    goalBias?: number
    goalTolerance?: number
    rewireRadius?: number
  }
}

export type ArmEvent =
  | { k: 'expand'; q: number[]; pq: number[] }
  | { k: 'rewire'; q: number[]; pq: number[]; opq: number[] }

export interface PlanArmResponse {
  found: boolean
  path: number[][]
  nodesExplored: number
  iterations: number
  events: ArmEvent[]
}

export interface ArmValidRequest extends ArmBase {
  obstacles: Obstacle[]
  linkThickness: number
  angles: number[]
}

/** Thrown when an engine call returns {"error": "..."}. */
export class EngineError extends Error {}

export function parseOrThrow<T>(raw: string): T {
  const v = JSON.parse(raw) as T | { error: string }
  if (v && typeof v === 'object' && 'error' in v && typeof (v as { error: unknown }).error === 'string') {
    throw new EngineError((v as { error: string }).error)
  }
  return v as T
}
