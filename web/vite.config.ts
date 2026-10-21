import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// base "./" so the build works from any static-hosting sub-path.
export default defineConfig({
  base: './',
  plugins: [react()],
})
