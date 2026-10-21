import { motion, useInView, useReducedMotion } from 'motion/react'
import { useEffect, useMemo, useRef, useState } from 'react'
import type { Algorithm } from '../engine/types'
import { ALGOS, ALGO_BY_ID, EASE_OUT } from '../lib/theme'
import { fmt } from '../lib/geometry'
import { CountUp } from './CountUp'

/** Shape written by `cargo run -p planner --release --bin bench` (bench/results.json). */
interface BenchReport {
  robotRadius: number
  gridSize: number
  seeds: number
  maps: number
  results: {
    mapId: string
    mapName: string
    algorithm: Algorithm
    runs: number
    successes: number
    meanTimeMs: number
    meanNodesExplored: number
    meanPathLength: number | null
  }[]
  summary: {
    algorithm: Algorithm
    runs: number
    successes: number
    success: string
    meanTimeMs: number
    meanNodesExplored: number
    meanPathLength: number | null
    meanPathRatioVsAstar: number | null
  }[]
}

type Metric = 'time' | 'nodes' | 'length'
const METRICS: { id: Metric; label: string; unit: string }[] = [
  { id: 'time', label: 'Time', unit: 'ms' },
  { id: 'nodes', label: 'Nodes', unit: '' },
  { id: 'length', label: 'Path length', unit: '' },
]

type Load = { state: 'loading' } | { state: 'missing' } | { state: 'ready'; data: BenchReport }

export function Benchmark() {
  const [load, setLoad] = useState<Load>({ state: 'loading' })
  const [metric, setMetric] = useState<Metric>('time')
  const ref = useRef<HTMLElement>(null)
  const inView = useInView(ref, { once: true, margin: '-80px' })
  const reduced = useReducedMotion()

  useEffect(() => {
    let alive = true
    fetch(`${import.meta.env.BASE_URL}bench-results.json`)
      .then((r) => (r.ok ? r.json() : Promise.reject(new Error(String(r.status)))))
      .then((data: BenchReport) => alive && setLoad({ state: 'ready', data }))
      .catch(() => alive && setLoad({ state: 'missing' }))
    return () => {
      alive = false
    }
  }, [])

  const matrix = useMemo(() => {
    if (load.state !== 'ready') return null
    const byMap = new Map<string, { id: string; name: string; cells: Partial<Record<Algorithm, BenchReport['results'][number]>> }>()
    for (const r of load.data.results) {
      const row = byMap.get(r.mapId) ?? { id: r.mapId, name: r.mapName, cells: {} }
      row.cells[r.algorithm] = r
      byMap.set(r.mapId, row)
    }
    const rows = [...byMap.values()]
    const val = (r: BenchReport['results'][number] | undefined): number | null => {
      if (!r) return null
      if (metric === 'time') return r.meanTimeMs
      if (metric === 'nodes') return r.meanNodesExplored
      return r.meanPathLength
    }
    const max = Math.max(1e-9, ...rows.flatMap((row) => ALGOS.map((a) => val(row.cells[a.id]) ?? 0)))
    return { rows, val, max }
  }, [load, metric])

  const show = inView || reduced

  return (
    <section className="bench" ref={ref} aria-labelledby="bench-title">
      <div className="bench-head">
        <h2 id="bench-title" className="h-section">Benchmark: 20 maps</h2>
        <p className="lede">
          Every planner on every fixed map, measured natively. A* runs once per map because it is deterministic; RRT and RRT* are averaged over
          {load.state === 'ready' ? ` ${load.data.seeds}` : ' 10'} seeds. Path length is averaged over successful runs only.
        </p>
      </div>

      {load.state === 'loading' && <div className="bench-empty">Loading results</div>}
      {load.state === 'missing' && (
        <div className="bench-empty">
          <p>No benchmark results have been published with this build yet.</p>
          <p className="mono-note">cargo run -p planner --release --bin bench, then copy bench/results.json to web/public/bench-results.json</p>
        </div>
      )}

      {load.state === 'ready' && matrix && (
        <>
          <div className="summary" role="table" aria-label="Summary per planner">
            <div className="summary-row summary-head" role="row">
              <span role="columnheader">Planner</span>
              <span role="columnheader">Success</span>
              <span role="columnheader" className="r">Mean time ms</span>
              <span role="columnheader" className="r">Mean nodes</span>
              <span role="columnheader" className="r">Length vs A*</span>
            </div>
            {load.data.summary.map((s, i) => {
              const meta = ALGO_BY_ID[s.algorithm]
              const rate = s.runs ? s.successes / s.runs : 0
              return (
                <motion.div
                  role="row"
                  key={s.algorithm}
                  className="summary-row"
                  style={{ ['--livery' as string]: meta.color }}
                  initial={reduced ? false : { opacity: 0, transform: 'translateY(12px)' }}
                  animate={show ? { opacity: 1, transform: 'translateY(0px)' } : undefined}
                  transition={{ duration: 0.45, ease: EASE_OUT, delay: 0.08 * i }}
                >
                  <span role="cell" className="who">
                    <span className="swatch" />
                    <span className="who-name">{meta.name}</span>
                  </span>
                  <span role="cell" className="success">
                    <span className="success-text">{s.success}</span>
                    <span className="success-track" aria-hidden>
                      <motion.span
                        className="success-fill"
                        initial={reduced ? false : { transform: 'scaleX(0)' }}
                        animate={show ? { transform: `scaleX(${rate})` } : undefined}
                        transition={{ duration: 0.9, ease: EASE_OUT, delay: 0.2 + 0.08 * i }}
                      />
                    </span>
                  </span>
                  <span role="cell" className="r">{show ? <CountUp value={s.meanTimeMs} decimals={2} duration={1100} /> : '0'}</span>
                  <span role="cell" className="r">{show ? <CountUp value={s.meanNodesExplored} duration={1100} /> : '0'}</span>
                  <span role="cell" className="r">
                    {s.meanPathRatioVsAstar === null ? '–' : show ? <CountUp value={s.meanPathRatioVsAstar} decimals={3} duration={1100} suffix="x" /> : '0'}
                  </span>
                </motion.div>
              )
            })}
          </div>

          <div className="matrix-controls">
            <div className="segmented" role="tablist" aria-label="Metric">
              {METRICS.map((m) => (
                <button
                  key={m.id}
                  role="tab"
                  type="button"
                  aria-selected={metric === m.id}
                  className={`seg${metric === m.id ? ' is-on' : ''}`}
                  onClick={() => setMetric(m.id)}
                >
                  {metric === m.id && <motion.span layoutId="bench-seg" className="seg-pill" transition={{ type: 'spring', duration: 0.4, bounce: 0.15 }} />}
                  <span className="seg-label">{m.label}</span>
                </button>
              ))}
            </div>
            <div className="legend">
              {ALGOS.map((a) => (
                <span key={a.id} className="legend-item" style={{ ['--livery' as string]: a.color }}>
                  <span className="swatch" />
                  {a.name}
                </span>
              ))}
            </div>
          </div>

          <div className="matrix" role="table" aria-label={`Per map ${metric}`}>
            {matrix.rows.map((row, ri) => (
              <motion.div
                role="row"
                key={row.id}
                className="matrix-row"
                initial={reduced ? false : { opacity: 0, transform: 'translateY(8px)' }}
                animate={show ? { opacity: 1, transform: 'translateY(0px)' } : undefined}
                transition={{ duration: 0.4, ease: EASE_OUT, delay: 0.25 + ri * 0.03 }}
              >
                <span role="rowheader" className="matrix-map">
                  <span className="tile-id">{row.id.toUpperCase()}</span>
                  {row.name}
                </span>
                <span role="cell" className="matrix-bars">
                  {ALGOS.map((a) => {
                    const c = row.cells[a.id]
                    const v = matrix.val(c)
                    const failedAll = c && c.successes === 0
                    return (
                      <span key={a.id} className="mbar" style={{ ['--livery' as string]: a.color }}>
                        <span className="mbar-track">
                          <span
                            className="mbar-fill"
                            style={{ transform: `scaleX(${show && v !== null ? Math.max(0.004, v / matrix.max) : 0})`, transitionDelay: `${0.3 + ri * 0.03}s` }}
                          />
                        </span>
                        <span className={`mbar-val${failedAll ? ' is-failed' : ''}`}>
                          {c ? (v === null ? 'DNF' : fmt(v, metric === 'time' ? 2 : 0)) : '–'}
                          {c && c.runs > 1 && <span className="mbar-runs"> {c.successes}/{c.runs}</span>}
                        </span>
                      </span>
                    )
                  })}
                </span>
              </motion.div>
            ))}
          </div>
          <p className="footnote">
            Robot radius {load.data.robotRadius}, grid {load.data.gridSize}. Times are native Rust on the machine that ran the benchmark, not in your browser.
          </p>
        </>
      )}
    </section>
  )
}
