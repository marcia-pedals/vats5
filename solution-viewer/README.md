# Solution Viewer

Fullstack app for browsing solver output, structured like `visualization/` but
backed by PostgreSQL instead of SQLite:

- **Frontend**: Vite + React + TanStack Router + Tailwind
- **Backend**: tRPC — Express locally (`server.ts`), a Vercel serverless
  function in production (`api/trpc/[trpc].ts`); both mount the same router
- **Database**: PostgreSQL via `pg`. Local cluster in development, Neon in
  production.

`/solutions` redirects to the newest pipeline run and shows its solved problem
instances, grouped by target stop set: service dates down, problem specs
across. The run selector switches between runs, and the run id is in the URL
(`/solutions/{gtfs_instance_id}`).

The rows come from `pipeline/run.py` — see `pipeline/` for how they get there.

## Local development

Everything below assumes the nix dev shell (direnv does this automatically).

```bash
bin/pg init                # start the shared cluster, write .env.local
cd solution-viewer
npm install
npm run migrate            # apply migrations/ (no-op if another checkout did)
cd ..
bin/sv start               # start the dev server
bin/sv url                 # print the URL
```

`bin/pg` manages a PostgreSQL cluster that is **shared by every checkout on the
machine**: it lives at `~/.local/share/vats5/pgdata` (override with
`VATS5_PGDATA`) and listens on port 15432 (override with `VATS5_PGPORT`), so a
solve recorded from one checkout shows up in every other one. Only one checkout
needs to have started it; `bin/pg init` in a fresh checkout finds the running
cluster and just writes that checkout's `.env.local`.

```
bin/pg {init|start|stop|restart|status|psql|url|destroy [--force]|logs}
bin/sv {start|stop|restart|status|url|logs}
```

`bin/pg destroy` wipes the shared data for all checkouts, so it asks for
confirmation unless given `--force`. The dev *servers* are still per-checkout —
`bin/sv` derives its ports from the checkout path as before.

The app reads `DATABASE_URL` from `solution-viewer/.env.local` (written by
`bin/pg init`, gitignored). See `.env.example` for the shape. A real
environment variable always wins over the file, so a one-off command can be
pointed at another database:

```bash
DATABASE_URL='postgresql://…neon.tech/…?sslmode=require' npm run migrate
```

## Migrations

Plain SQL files in `migrations/`, applied in filename order by
`npm run migrate`. Each file runs in a transaction alongside its bookkeeping
insert into `schema_migrations`, so a failed migration leaves nothing behind.
Migrations are apply-only — to change something, add a new numbered file.

## Deploying to Vercel

1. Create the Neon database and run the migrations against it:
   ```bash
   DATABASE_URL='<neon-connection-string>' npm run migrate
   ```
2. Create the Vercel project with **Root Directory** set to `solution-viewer`
   (the repo has other top-level apps).
3. Set `DATABASE_URL` to the Neon connection string in the Vercel project's
   environment variables, for every environment you deploy.
4. Deploy. `vercel.json` builds the Vite app to `dist/`, routes `/trpc/*` to
   the serverless function, and falls back to `index.html` for client routes.

Neon's pooled (`-pooler`) connection string is the right one for serverless
functions — each invocation opens its own connection, and the pooler keeps that
from exhausting the database's connection limit.

## Structure

```
solution-viewer/
├── api/trpc/[trpc].ts     # Vercel serverless entrypoint (fetch adapter)
├── server.ts              # local Express + tRPC server
├── migrations/            # numbered .sql files
├── scripts/migrate.ts     # migration runner
├── src/
│   ├── server/
│   │   ├── db.ts          # pg pool + queries, zod-validated
│   │   ├── env.ts         # loads .env.local in development
│   │   └── trpc.ts        # tRPC router
│   ├── client/trpc.ts     # tRPC client
│   └── routes/            # TanStack Router file routes
├── vercel.json
└── vite.config.ts
```
