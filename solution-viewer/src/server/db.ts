import { Pool, type PoolConfig } from "pg";
import { z } from "zod";
import { type Run, SolutionSchema, type Solution } from "../schemas";

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

// --- Row schemas ---
//
// The shapes of the data itself live in src/schemas.ts, shared with the client.
// What is here is how a row comes back from postgres.

// One GTFS fetch, i.e. one pipeline run. `fetched_at` arrives as a Date and is
// sent to the client as an ISO string.
const RunRowSchema = z.object({
  gtfs_instance_id: z.string(),
  fetched_at: z.date(),
});

// --- Queries ---

/** Every GTFS fetch that the pipeline has run against, newest first. */
export async function listRuns(): Promise<Run[]> {
  const rows = await query(`
    SELECT gtfs_instance_id, fetched_at
    FROM gtfs_instance
    ORDER BY fetched_at DESC
  `);
  return z
    .array(RunRowSchema)
    .parse(rows)
    .map((row) => ({ ...row, fetched_at: row.fetched_at.toISOString() }));
}

/** Every problem instance solved against one GTFS fetch. */
export async function listSolutions(gtfsInstanceId: string): Promise<Solution[]> {
  const rows = await query(
    `
    SELECT instance.problem_instance_id,
           instance.problem_spec_id,
           spec.title AS spec_title,
           spec.target_stops_id,
           stops.title AS target_stops_title,
           instance.service_date,
           instance.gtfs_instance_id,
           instance.data
    FROM problem_instance AS instance
    JOIN problem_spec AS spec
      ON spec.problem_spec_id = instance.problem_spec_id
     AND spec.gtfs_source_id = instance.gtfs_source_id
    JOIN target_stops AS stops
      ON stops.target_stops_id = spec.target_stops_id
     AND stops.gtfs_source_id = spec.gtfs_source_id
    WHERE instance.gtfs_instance_id = $1
    ORDER BY stops.title, instance.service_date, instance.problem_spec_id
  `,
    [gtfsInstanceId]
  );
  return z.array(SolutionSchema).parse(rows);
}
