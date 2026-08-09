import { Pool, type PoolConfig } from "pg";
import { z } from "zod";

/**
 * Connection settings from DATABASE_URL. Local development talks to a plain,
 * unencrypted cluster started by bin/pg; anything remote (Neon) requires TLS
 * with a verified certificate.
 */
export function getConnectionConfig(): PoolConfig {
  const connectionString = process.env.DATABASE_URL;
  if (!connectionString) {
    throw new Error("DATABASE_URL is not set (see solution-viewer/README.md)");
  }
  const isLocal = /@(localhost|127\.0\.0\.1)[:/]/.test(connectionString);
  return { connectionString, ssl: isLocal ? false : { rejectUnauthorized: true } };
}

// A single pool per process. On Vercel this module is reused across warm
// invocations of the same serverless function, so the pool is kept small.
let pool: Pool | undefined;

function getPool(): Pool {
  if (!pool) {
    pool = new Pool({ ...getConnectionConfig(), max: 5 });
  }
  return pool;
}

async function query(sql: string, params: unknown[] = []): Promise<unknown[]> {
  const result = await getPool().query(sql, params);
  return result.rows;
}

// --- Schemas ---

const HelloWorldRowSchema = z.object({
  id: z.number(),
  message: z.string(),
  created_at: z.date(),
});

const HelloWorldSchema = z.object({
  id: z.number(),
  message: z.string(),
  created_at: z.string(),
});
export type HelloWorld = z.infer<typeof HelloWorldSchema>;

// --- Queries ---

export async function listHelloWorld(): Promise<HelloWorld[]> {
  const rows = await query("SELECT id, message, created_at FROM hello_world ORDER BY id");
  return z
    .array(HelloWorldRowSchema)
    .parse(rows)
    .map((row) => ({ ...row, created_at: row.created_at.toISOString() }));
}
