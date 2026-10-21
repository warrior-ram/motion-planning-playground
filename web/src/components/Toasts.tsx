import { AnimatePresence, motion, useReducedMotion } from 'motion/react'
import { EASE_OUT } from '../lib/theme'

export interface Toast { id: number; text: string; tone: 'info' | 'danger' }

export function Toasts({ items }: { items: Toast[] }) {
  const reduced = useReducedMotion()
  return (
    <div className="toasts" aria-live="polite" role="status">
      <AnimatePresence initial={false}>
        {items.map((t) => (
          <motion.div
            key={t.id}
            layout={!reduced}
            className={`toast toast-${t.tone}`}
            initial={reduced ? { opacity: 0 } : { opacity: 0, transform: 'translateY(100%) scale(0.96)' }}
            animate={{ opacity: 1, transform: 'translateY(0%) scale(1)' }}
            exit={reduced ? { opacity: 0 } : { opacity: 0, transform: 'translateY(30%) scale(0.98)', transition: { duration: 0.18, ease: EASE_OUT } }}
            transition={{ duration: 0.34, ease: [0.32, 0.72, 0, 1] }}
          >
            {t.text}
          </motion.div>
        ))}
      </AnimatePresence>
    </div>
  )
}
