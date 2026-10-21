import { AnimatePresence, motion, MotionConfig, useReducedMotion } from 'motion/react'
import { useCallback, useEffect, useRef, useState } from 'react'
import { getBenchmarkMaps, initPlannerEngine } from './engine/planner'
import { initKinematicsEngine } from './engine/kinematics'
import type { BenchMap } from './engine/types'
import { EASE_OUT } from './lib/theme'
import { RobotMode } from './components/RobotMode'
import { ArmMode } from './components/ArmMode'
import { Benchmark } from './components/Benchmark'
import { Toasts, type Toast } from './components/Toasts'

type Mode = 'robot' | 'arm'
type Boot = { state: 'loading' } | { state: 'error'; message: string } | { state: 'ready'; maps: BenchMap[] }

const MODES: { id: Mode; label: string }[] = [
  { id: 'robot', label: 'Mobile robot' },
  { id: 'arm', label: 'Robot arm' },
]

const NOTES = [
  {
    name: 'A*',
    color: '#FF7A45',
    text: 'Searches a grid of 10-unit cells, always expanding the cell with the lowest cost so far plus an octile estimate of the distance left. On the grid it finds the shortest path, but it must touch every cell that could be on it.',
  },
  {
    name: 'RRT',
    color: '#34D1C4',
    text: 'Grows a tree by sampling random points and stepping toward each from the nearest node. It covers open space fast and needs no grid, but the first path it finds is rarely short.',
  },
  {
    name: 'RRT*',
    color: '#E8E15A',
    text: 'RRT plus rewiring: each new node picks the cheapest nearby parent and offers itself as a cheaper parent to its neighbours. The white flashes are rewires. Paths straighten as the tree grows.',
  },
]

export default function App() {
  const [boot, setBoot] = useState<Boot>({ state: 'loading' })
  const [mode, setMode] = useState<Mode>('robot')
  const [toasts, setToasts] = useState<Toast[]>([])
  const toastId = useRef(0)
  const reduced = useReducedMotion()

  useEffect(() => {
    Promise.all([initPlannerEngine(), initKinematicsEngine()])
      .then(() => setBoot({ state: 'ready', maps: getBenchmarkMaps() }))
      .catch((e: unknown) => setBoot({ state: 'error', message: e instanceof Error ? e.message : String(e) }))
  }, [])

  const toast = useCallback((text: string, tone: 'info' | 'danger' = 'info') => {
    const id = ++toastId.current
    setToasts((t) => [...t.slice(-2), { id, text, tone }])
    setTimeout(() => setToasts((t) => t.filter((x) => x.id !== id)), 3200)
  }, [])

  const enter = (delay: number) =>
    reduced
      ? { initial: { opacity: 0 }, animate: { opacity: 1 }, transition: { duration: 0.2 } }
      : {
          initial: { opacity: 0, transform: 'translateY(14px)', filter: 'blur(4px)' },
          animate: { opacity: 1, transform: 'translateY(0px)', filter: 'blur(0px)' },
          transition: { duration: 0.6, ease: EASE_OUT, delay },
        }

  return (
    <MotionConfig reducedMotion="user">
      <div className="page">
        <header className="masthead">
          <motion.div className="brand" {...enter(0)}>
            <span className="brand-mark" aria-hidden>
              <span />
              <span />
              <span />
            </span>
            <h1 className="brand-name">Motion planning playground</h1>
          </motion.div>
          <motion.p className="claim" {...enter(0.08)}>
            A*, RRT and RRT* written in Rust, compiled to WebAssembly and running in this tab. Draw walls, move the start and the goal, and race them.
          </motion.p>
          <motion.nav className="modes" aria-label="Mode" {...enter(0.16)}>
            <div className="segmented segmented-lg" role="tablist">
              {MODES.map((m) => (
                <button
                  key={m.id}
                  type="button"
                  role="tab"
                  aria-selected={mode === m.id}
                  className={`seg${mode === m.id ? ' is-on' : ''}`}
                  onClick={() => setMode(m.id)}
                >
                  {mode === m.id && <motion.span layoutId="mode-seg" className="seg-pill" transition={{ type: 'spring', duration: 0.4, bounce: 0.15 }} />}
                  <span className="seg-label">{m.label}</span>
                </button>
              ))}
            </div>
          </motion.nav>
        </header>

        <motion.main className="main" {...enter(0.24)}>
          {boot.state === 'loading' && (
            <div className="boot" role="status">
              <span className="boot-bar" aria-hidden />
              Loading the planners
            </div>
          )}
          {boot.state === 'error' && (
            <div className="boot boot-error" role="alert">
              <strong>The planners failed to load.</strong> {boot.message}. Reload the page; if it keeps failing, this browser may not support WebAssembly.
            </div>
          )}
          {boot.state === 'ready' && (
            <AnimatePresence mode="wait" initial={false}>
              <motion.div
                key={mode}
                initial={reduced ? { opacity: 0 } : { opacity: 0, transform: 'translateY(8px)', filter: 'blur(3px)' }}
                animate={{ opacity: 1, transform: 'translateY(0px)', filter: 'blur(0px)' }}
                exit={reduced ? { opacity: 0 } : { opacity: 0, transform: 'translateY(-6px)', filter: 'blur(3px)', transition: { duration: 0.14 } }}
                transition={{ duration: 0.28, ease: EASE_OUT }}
              >
                {mode === 'robot' ? <RobotMode maps={boot.maps} toast={toast} /> : <ArmMode toast={toast} />}
              </motion.div>
            </AnimatePresence>
          )}
        </motion.main>

        <section className="notes" aria-labelledby="notes-title">
          <h2 id="notes-title" className="h-section">How each planner works</h2>
          <div className="notes-grid">
            {NOTES.map((n, i) => (
              <motion.article
                key={n.name}
                className="note"
                style={{ ['--livery' as string]: n.color }}
                initial={reduced ? false : { opacity: 0, transform: 'translateY(16px)' }}
                whileInView={{ opacity: 1, transform: 'translateY(0px)' }}
                viewport={{ once: true, margin: '-60px' }}
                transition={{ duration: 0.5, ease: EASE_OUT, delay: i * 0.07 }}
              >
                <h3 className="note-name">{n.name}</h3>
                <p>{n.text}</p>
              </motion.article>
            ))}
          </div>
        </section>

        <Benchmark />

        <footer className="foot">
          <p>
            Planning time is measured with <code>performance.now()</code> around each WebAssembly call, on your device. Random planners are seeded, so the same seed gives the same tree.
          </p>
        </footer>
      </div>
      <Toasts items={toasts} />
    </MotionConfig>
  )
}
