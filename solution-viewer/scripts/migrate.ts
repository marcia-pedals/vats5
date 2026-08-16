import * as fs from "node:fs/promises";
import * as path from "node:path";
import { fileURLToPath } from "node:url";
import { Client } from "pg";
import "../src/server/env";
import { getConnectionConfig } from "../src/server/db";

// Applies every .sql file in migrations/ that hasn't been applied yet, in
// filename order. Each migration runs inside a transaction together with the
// bookkeeping insert, so a failure leaves the database untouched.
//
//   npm run migrate                       # local db from .env.local
//   DATABASE_URL=<neon-url> npm run migrate

const MIGRATIONS_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../migrations");

async function main(): Promise<void> {
  const files = (await fs.readdir(MIGRATIONS_DIR)).filter((f) => f.endsWith(".sql")).sort();

  const client = new Client(getConnectionConfig());
  await client.connect();
  try {
    await client.query(`
      CREATE TABLE IF NOT EXISTS schema_migrations (
        filename TEXT PRIMARY KEY,
        applied_at TIMESTAMPTZ NOT NULL DEFAULT now()
      )
    `);

    const applied = new Set(
      (await client.query<{ filename: string }>("SELECT filename FROM schema_migrations")).rows.map(
        (row) => row.filename
      )
    );

    const pending = files.filter((file) => !applied.has(file));
    if (pending.length === 0) {
      console.log(`no pending migrations (${applied.size} already applied)`);
      return;
    }

    for (const file of pending) {
      const sql = await fs.readFile(path.join(MIGRATIONS_DIR, file), "utf8");
      process.stdout.write(`applying ${file}... `);
      await client.query("BEGIN");
      try {
        await client.query(sql);
        await client.query("INSERT INTO schema_migrations (filename) VALUES ($1)", [file]);
        await client.query("COMMIT");
      } catch (error) {
        await client.query("ROLLBACK");
        console.log("failed");
        throw error;
      }
      console.log("ok");
    }
    console.log(`applied ${pending.length} migration(s)`);
  } finally {
    await client.end();
  }
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});
