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

// One span of the hierarchical timing trace pipeline/run.py records for a
// solve. `start_seconds` is relative to the start of the root span.
export type TraceNode = {
  name: string;
  start_seconds: number;
  duration_seconds: number;
  metadata?: Record<string, unknown>;
  children?: TraceNode[];
};

const TraceNodeSchema: z.ZodType<TraceNode> = z.lazy(() =>
  z.object({
    name: z.string(),
    start_seconds: z.number(),
    duration_seconds: z.number(),
    metadata: z.record(z.unknown()).optional(),
    children: z.array(TraceNodeSchema).optional(),
  })
);

// What pipeline/run.py stores in problem_instance.data. `status` is "solved",
// "timeout", or one of the failure statuses; the other fields depend on it.
// `trace` is absent for rows written before it was recorded.
const SolutionDataSchema = z
  .object({
    status: z.string(),
    optimal_duration_seconds: z.number().optional(),
    timeout_seconds: z.number().optional(),
    returncode: z.number().nullable().optional(),
    trace: TraceNodeSchema.optional(),
  })
  .passthrough();

const SolutionRowSchema = z.object({
  problem_instance_id: z.string(),
  problem_spec_id: z.string(),
  spec_title: z.string(),
  target_stops_id: z.string(),
  target_stops_title: z.string(),
  service_date: z.string(),
  gtfs_instance_id: z.string(),
  data: SolutionDataSchema,
});
export type Solution = z.infer<typeof SolutionRowSchema>;

// One GTFS fetch, i.e. one pipeline run.
const RunRowSchema = z.object({
  gtfs_instance_id: z.string(),
  fetched_at: z.date(),
});

const RunSchema = RunRowSchema.extend({ fetched_at: z.string() });
export type Run = z.infer<typeof RunSchema>;

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
  return z.array(SolutionRowSchema).parse(rows);
}
