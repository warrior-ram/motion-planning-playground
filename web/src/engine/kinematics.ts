import initKin, { fk, ik, plan_arm, arm_config_valid } from '../wasm/kinematics/kinematics'
import {
  parseOrThrow,
  type ArmBase,
  type ArmValidRequest,
  type FkResponse,
  type IkRequest,
  type IkResponse,
  type PlanArmRequest,
  type PlanArmResponse,
  type Point,
} from './types'

let ready: Promise<void> | null = null

export function initKinematicsEngine(): Promise<void> {
  if (!ready) ready = initKin().then(() => undefined)
  return ready
}

export function forwardKinematics(arm: ArmBase, angles: number[]): Point[] {
  return parseOrThrow<FkResponse>(fk(JSON.stringify({ ...arm, angles }))).joints
}

export function inverseKinematics(req: IkRequest): IkResponse {
  return parseOrThrow<IkResponse>(ik(JSON.stringify(req)))
}

export interface TimedArmPlan extends PlanArmResponse { timeMs: number }

export function planArm(req: PlanArmRequest): TimedArmPlan {
  const body = JSON.stringify(req)
  const t0 = performance.now()
  const raw = plan_arm(body)
  const timeMs = performance.now() - t0
  return { ...parseOrThrow<PlanArmResponse>(raw), timeMs }
}

export function armConfigValid(req: ArmValidRequest): boolean {
  return parseOrThrow<{ valid: boolean }>(arm_config_valid(JSON.stringify(req))).valid
}
