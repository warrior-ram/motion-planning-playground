import { useEffect, useRef, useState } from 'react'
import { useReducedMotion } from 'motion/react'
import { fmt } from '../lib/geometry'

/** Number that counts toward its value with an exponential ease-out. */
export function CountUp({ value, decimals = 0, duration = 700, suffix = '' }: { value: number | null; decimals?: number; duration?: number; suffix?: string }) {
  const reduced = useReducedMotion()
  const [shown, setShown] = useState(value ?? 0)
  // Last value drawn, so a new target counts from wherever the number currently is.
  const shownRef = useRef(shown)

  useEffect(() => {
    if (value === null) return
    if (reduced) {
      shownRef.current = value
      return
    }
    const from = shownRef.current
    const t0 = performance.now()
    let raf = 0
    const tick = (now: number) => {
      const t = Math.min(1, (now - t0) / duration)
      const e = t >= 1 ? 1 : 1 - Math.pow(2, -10 * t)
      const v = from + (value - from) * e
      shownRef.current = v
      setShown(v)
      if (t < 1) raf = requestAnimationFrame(tick)
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [value, duration, reduced])

  if (value === null) return <span className="num num-empty">&ndash;</span>
  return (
    <span className="num">
      {fmt(reduced ? value : shown, decimals)}
      {suffix}
    </span>
  )
}
