import { AnimatePresence, LayoutGroup, motion, useReducedMotion } from 'motion/react'
import type { Algorithm } from '../engine/types'
import { ALGO_BY_ID, EASE_OUT } from '../lib/theme'
import { CountUp } from './CountUp'
import type { RunPhase } from './RobotCanvas'

export interface TowerRow {
  algo: Algorithm
  found: boolean
  timeMs: number
  nodes: number
  length: number
  iterations: number
  phase: RunPhase
  frac: number
}

const PHASE_LABEL: Record<RunPhase, string> = {
  explore: 'Exploring',
  path: 'Path found',
  drive: 'Driving',
  arrived: 'Arrived',
  failed: 'No path',
}

/**
 * Timing tower. Rows sort by finish (shortest path first, failures last) and
 * re-sort with layout animation; the best value per column lights purple.
 */
export function TimingTower({ rows, maxIterations }: { rows: TowerRow[]; maxIterations: number }) {
  const reduced = useReducedMotion()
  const sorted = [...rows].sort((a, b) => {
    if (a.found !== b.found) return a.found ? -1 : 1
    return a.found ? a.length - b.length : 0
  })
  const ok = rows.filter((r) => r.found)
  const best = {
    time: ok.length > 1 ? Math.min(...ok.map((r) => r.timeMs)) : null,
    nodes: ok.length > 1 ? Math.min(...ok.map((r) => r.nodes)) : null,
    length: ok.length > 1 ? Math.min(...ok.map((r) => r.length)) : null,
  }
  const settled = rows.every((r) => r.phase === 'arrived' || r.phase === 'failed')

  return (
    <section className="tower" aria-label="Results">
      <div className="tower-head" role="row">
        <span>Pos</span>
        <span>Planner</span>
        <span className="r">Time ms</span>
        <span className="r">Nodes</span>
        <span className="r">Length</span>
      </div>
      <LayoutGroup>
        <ol className="tower-rows">
          <AnimatePresence initial={false} mode="popLayout">
            {sorted.map((r, i) => {
              const meta = ALGO_BY_ID[r.algo]
              const done = r.phase === 'arrived' || r.phase === 'failed'
              return (
                <motion.li
                  key={r.algo}
                  layout={!reduced}
                  className={`tower-row${r.found ? '' : ' is-failed'}`}
                  style={{ ['--livery' as string]: meta.color }}
                  initial={reduced ? { opacity: 0 } : { opacity: 0, transform: 'translateX(-14px)' }}
                  animate={{ opacity: 1, transform: 'translateX(0px)' }}
                  exit={reduced ? { opacity: 0 } : { opacity: 0, transform: 'translateX(14px)', transition: { duration: 0.16 } }}
                  transition={{ layout: { type: 'spring', duration: 0.5, bounce: 0.18 }, duration: 0.32, ease: EASE_OUT, delay: i * 0.05 }}
                >
                  <span className="pos">{settled && r.found ? i + 1 : <span className="pos-dash" />}</span>
                  <span className="who">
                    <span className="swatch" />
                    <span className="who-name">{meta.name}</span>
                    <span className="who-phase" key={r.phase}>
                      {PHASE_LABEL[r.phase]}
                    </span>
                  </span>
                  <span className={`r cell${best.time !== null && r.found && r.timeMs === best.time && done ? ' best' : ''}`}>
                    <CountUp value={r.timeMs} decimals={r.timeMs < 10 ? 2 : 1} />
                  </span>
                  <span className={`r cell${best.nodes !== null && r.found && r.nodes === best.nodes && done ? ' best' : ''}`}>
                    <CountUp value={r.nodes} />
                  </span>
                  <span className={`r cell${best.length !== null && r.found && r.length === best.length && done ? ' best' : ''}`}>
                    {r.found ? <CountUp value={r.length} decimals={0} /> : <span className="dnf">DNF</span>}
                  </span>
                  <span className="bar" aria-hidden>
                    <span className="bar-fill" style={{ transform: `scaleX(${r.phase === 'explore' ? r.frac : 1})` }} />
                  </span>
                  {!r.found && done && (
                    <span className="row-note">
                      {r.algo === 'astar'
                        ? 'The goal is walled off at this robot size.'
                        : `No path in ${r.iterations.toLocaleString('en-US')} of ${maxIterations.toLocaleString('en-US')} iterations.`}
                    </span>
                  )}
                </motion.li>
              )
            })}
          </AnimatePresence>
        </ol>
      </LayoutGroup>
    </section>
  )
}
