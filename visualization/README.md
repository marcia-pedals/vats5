# Visualization

Simple fullstack setup with:
- **Frontend**: Vite + React + TypeScript
- **Backend**: Express + tRPC
- **Communication**: tRPC for type-safe APIs

## Development

```bash
npm install
npm run dev
```

This starts both:
- Express server on `http://0.0.0.0:4000` (tRPC API)
- Vite dev server on `http://0.0.0.0:3000` (Frontend, proxies /trpc to Express)

## Production

```bash
npm run build
```

Then serve the `dist/` folder from your Express server.

## Structure

```
visualization/
├── server.js              # Express + tRPC server
├── src/
│   ├── server/
│   │   └── trpc.ts       # tRPC router (backend logic)
│   └── client/
│       ├── App.tsx       # React app
│       └── trpc.ts       # tRPC client setup
└── vite.config.ts        # Vite config (proxies to Express)
```
