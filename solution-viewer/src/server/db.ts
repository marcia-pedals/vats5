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

// One stop of the problem, as iterative_expansion reports it. `id` is the GTFS
// stop id, which is what a path step refers to.
const SolutionStopSchema = z.object({
  id: z.string(),
  name: z.string(),
  lat: z.number(),
  lon: z.number(),
  required: z.boolean(),
});
export type SolutionStop = z.infer<typeof SolutionStopSchema>;

// One GTFS route+direction a leg of the solution path travels on. The colors
// are GTFS route_color/route_text_color, without the "#", and may be empty.
const SolutionRouteSchema = z.object({
  id: z.string(),
  name: z.string(),
  color: z.string(),
  text_color: z.string(),
});
export type SolutionRoute = z.infer<typeof SolutionRouteSchema>;

// One leg of the solution path. `route_direction_id` is null for a walk.
const PathStepSchema = z.object({
  origin_stop_id: z.string(),
  destination_stop_id: z.string(),
  depart_time: z.number(),
  arrive_time: z.number(),
  is_flex: z.number(),
  route_direction_id: z.string().nullable().optional(),
});
export type PathStep = z.infer<typeof PathStepSchema>;

// The tour the solver settled on, without its synthetic START/END edges.
// `steps` collapses each run of consecutive steps on one trip into a single
// leg; `original_steps` keeps every stop the path passes through.
const SolutionPathSchema = z.object({
  steps: z.array(PathStepSchema),
  original_steps: z.array(PathStepSchema),
  duration: z.number(),
});
export type SolutionPath = z.infer<typeof SolutionPathSchema>;

// What pipeline/run.py stores in problem_instance.data. `status` is "solved",
// "timeout", or one of the failure statuses; the other fields depend on it.
// `trace` is absent for rows written before it was recorded, and so are
// `stops`/`solution_path`/`routes`. Only a solve has a path; `routes` covers
// exactly the routes that path uses.
const SolutionDataSchema = z
  .object({
    status: z.string(),
    optimal_duration_seconds: z.number().optional(),
    timeout_seconds: z.number().optional(),
    returncode: z.number().nullable().optional(),
    trace: TraceNodeSchema.optional(),
    stops: z.array(SolutionStopSchema).optional(),
    routes: z.array(SolutionRouteSchema).optional(),
    solution_path: SolutionPathSchema.optional(),
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
