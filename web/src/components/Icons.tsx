// One stroke system: 20px box, 1.6 stroke, round caps.
const base = { width: 20, height: 20, viewBox: '0 0 20 20', fill: 'none', stroke: 'currentColor', strokeWidth: 1.6, strokeLinecap: 'round' as const, strokeLinejoin: 'round' as const, 'aria-hidden': true }

export const IconRect = () => (<svg {...base}><rect x="3.5" y="5" width="13" height="10" rx="1.5" /></svg>)
export const IconCircle = () => (<svg {...base}><circle cx="10" cy="10" r="6.5" /></svg>)
export const IconMove = () => (<svg {...base}><path d="M10 3v14M3 10h14M10 3l-2.2 2.2M10 3l2.2 2.2M10 17l-2.2-2.2M10 17l2.2-2.2M3 10l2.2-2.2M3 10l2.2 2.2M17 10l-2.2-2.2M17 10l-2.2 2.2" /></svg>)
export const IconErase = () => (<svg {...base}><path d="M8 16h8M4.5 12.5l6-6.5a1.5 1.5 0 0 1 2.1 0l2.4 2.4a1.5 1.5 0 0 1 0 2.1L10 16H7.5l-3-3a.7.7 0 0 1 0-.5z" /></svg>)
export const IconPlay = () => (<svg {...base}><path d="M6.5 4.5v11l9-5.5z" fill="currentColor" stroke="none" /></svg>)
export const IconFlag = () => (<svg {...base}><path d="M5 17V3.5M5 4h9.5l-2 3.25 2 3.25H5" /></svg>)
export const IconSkip = () => (<svg {...base}><path d="M5 5l6 5-6 5zM12.5 5v10" /></svg>)
export const IconReset = () => (<svg {...base}><path d="M4 10a6 6 0 1 0 1.8-4.3M4 3.5v3h3" /></svg>)
export const IconArm = () => (<svg {...base}><path d="M4 16h6M7 16l2-6 5-3M14 7l2.5 1.5" /><circle cx="9" cy="10" r="1.2" /><circle cx="14" cy="7" r="1.2" /></svg>)
export const IconSwap = () => (<svg {...base}><path d="M4 7h11l-3-3M16 13H5l3 3" /></svg>)
