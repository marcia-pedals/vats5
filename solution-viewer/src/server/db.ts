import { Pool, type PoolConfig } from "pg";
import { z } from "zod";
import {
  type Run,
  type Solution,
  SolutionSchema,
  type TargetStops,
  TargetStopsSchema,
} from "../schemas.ts";

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

/**
 * Every target stop set something has actually been solved for. The set is the
 * outer axis the viewer navigates by, so one with no instances behind it would
 * only be an entry that leads nowhere.
 */
export async function listTargetStops(): Promise<TargetStops[]> {
  const rows = await query(`
    SELECT DISTINCT stops.target_stops_id, stops.title
    FROM target_stops AS stops
    JOIN problem_spec AS spec
      ON spec.target_stops_id = stops.target_stops_id
     AND spec.gtfs_source_id = stops.gtfs_source_id
    JOIN problem_instance AS instance
      ON instance.problem_spec_id = spec.problem_spec_id
     AND instance.gtfs_source_id = spec.gtfs_source_id
    ORDER BY stops.title
  `);
  return z.array(TargetStopsSchema).parse(rows);
}

/**
 * The GTFS fetches that one target stop set was solved against, newest first.
 * Scoped to the set rather than every fetch there is, so stepping through runs
 * never lands on one that has nothing to show for the set being viewed.
 */
export async function listRuns(targetStopsId: string): Promise<Run[]> {
  const rows = await query(
    `
    SELECT DISTINCT gtfs.gtfs_instance_id, gtfs.fetched_at
    FROM gtfs_instance AS gtfs
    JOIN problem_instance AS instance
      ON instance.gtfs_instance_id = gtfs.gtfs_instance_id
     AND instance.gtfs_source_id = gtfs.gtfs_source_id
    JOIN problem_spec AS spec
      ON spec.problem_spec_id = instance.problem_spec_id
     AND spec.gtfs_source_id = instance.gtfs_source_id
    WHERE spec.target_stops_id = $1
    ORDER BY gtfs.fetched_at DESC
  `,
    [targetStopsId]
  );
  return z
    .array(RunRowSchema)
    .parse(rows)
    .map((row) => ({ ...row, fetched_at: row.fetched_at.toISOString() }));
}

/** Every problem instance for one target stop set solved against one GTFS fetch. */
export async function listSolutions(
  gtfsInstanceId: string,
  targetStopsId: string
): Promise<Solution[]> {
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
      AND spec.target_stops_id = $2
    ORDER BY instance.service_date, instance.problem_spec_id
  `,
    [gtfsInstanceId, targetStopsId]
  );
  return z.array(SolutionSchema).parse(rows);
}
