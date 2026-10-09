import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// The backend (uvicorn) listens on 127.0.0.1:8000. Proxying keeps the browser
// on one origin, so no CORS configuration is needed.
const backend = 'http://127.0.0.1:8000';
const proxy = {
  '/api': { target: backend, changeOrigin: true },
  '/ws': { target: backend, ws: true, changeOrigin: true },
};

export default defineConfig({
  plugins: [react()],
  server: { host: '127.0.0.1', port: 5173, strictPort: true, proxy },
  preview: { host: '127.0.0.1', port: 4173, strictPort: true, proxy },
  build: { chunkSizeWarningLimit: 900 },
});
