import { motion, useReducedMotion } from 'motion/react'
import { useRef } from 'react'
import type { BenchMap } from '../engine/types'
import { EASE_OUT } from '../lib/theme'

function Thumb({ map }: { map: BenchMap }) {
  return (
    <svg viewBox={`0 0 ${map.width} ${map.height}`} className="thumb-svg" aria-hidden>
      <rect width={map.width} height={map.height} className="thumb-ground" />
      {map.obstacles.map((o, i) =>
        o.kind === 'rect' ? (
          <rect key={i} x={o.x} y={o.y} width={o.w} height={o.h} rx={6} className="thumb-wall" />
        ) : (
          <circle key={i} cx={o.x} cy={o.y} r={o.r} className="thumb-wall" />
        ),
      )}
      <circle cx={map.start.x} cy={map.start.y} r={18} className="thumb-start" />
      <circle cx={map.goal.x} cy={map.goal.y} r={18} className="thumb-goal" />
    </svg>
  )
}

/** Horizontal rail of the 20 benchmark maps; the hovered or focused tile lifts and the rest dim. */
export function PresetRail({ maps, activeId, onPick }: { maps: BenchMap[]; activeId: string | null; onPick: (m: BenchMap) => void }) {
  const reduced = useReducedMotion()
  const railRef = useRef<HTMLDivElement>(null)

  const nudge = (dir: number) => {
    const el = railRef.current
    if (!el) return
    el.scrollBy({ left: dir * el.clientWidth * 0.8, behavior: reduced ? 'auto' : 'smooth' })
  }

  return (
    <section className="rail" aria-label="Benchmark maps">
      <div className="rail-head">
        <h2 className="h-small">Maps</h2>
        <p className="rail-sub">The 20 fixed benchmark maps. Pick one, or draw your own.</p>
        <div className="rail-nav">
          <button type="button" className="btn-ghost icon-only" aria-label="Scroll maps left" onClick={() => nudge(-1)}>
            <svg width="20" height="20" viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" aria-hidden><path d="M12 4l-6 6 6 6" /></svg>
          </button>
          <button type="button" className="btn-ghost icon-only" aria-label="Scroll maps right" onClick={() => nudge(1)}>
            <svg width="20" height="20" viewBox="0 0 20 20" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" aria-hidden><path d="M8 4l6 6-6 6" /></svg>
          </button>
        </div>
      </div>
      <div className="rail-track" ref={railRef}>
        {maps.map((m, i) => (
          <motion.button
            type="button"
            key={m.id}
            className={`tile${m.id === activeId ? ' is-active' : ''}`}
            aria-pressed={m.id === activeId}
            onClick={() => onPick(m)}
            initial={reduced ? false : { opacity: 0, transform: 'translateY(10px)' }}
            animate={{ opacity: 1, transform: 'translateY(0px)' }}
            transition={{ duration: 0.4, ease: EASE_OUT, delay: 0.35 + Math.min(i, 10) * 0.035 }}
          >
            <Thumb map={m} />
            <span className="tile-meta">
              <span className="tile-id">{m.id.toUpperCase()}</span>
              <span className="tile-name">{m.name}</span>
            </span>
          </motion.button>
        ))}
      </div>
    </section>
  )
}
