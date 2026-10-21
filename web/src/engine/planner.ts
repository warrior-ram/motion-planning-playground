import initPlanner, { plan, benchmark_maps } from '../wasm/planner/planner'
import { parseOrThrow, type BenchMap, type PlanRequest, type PlanResponse } from './types'

let ready: Promise<void> | null = null

/** Initialise the planner WASM module once. */
export function initPlannerEngine(): Promise<void> {
  if (!ready) ready = initPlanner().then(() => undefined)
  return ready
}

export interface TimedPlan extends PlanResponse { timeMs: number }

/** Run a planner; time is measured here with performance.now() per the contract. */
export function runPlan(req: PlanRequest): TimedPlan {
  const body = JSON.stringify(req)
  const t0 = performance.now()
  const raw = plan(body)
  const timeMs = performance.now() - t0
  return { ...parseOrThrow<PlanResponse>(raw), timeMs }
}

export function getBenchmarkMaps(): BenchMap[] {
  return parseOrThrow<BenchMap[]>(benchmark_maps())
}
