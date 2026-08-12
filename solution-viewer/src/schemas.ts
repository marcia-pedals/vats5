import { z } from "zod";

/**
 * The shapes of the data the pipeline stores in `problem_instance.data`, shared
 * by the server (which parses rows with these schemas) and the client (which
 * renders them). Nothing here may import `pg`, since it is bundled for the
 * browser.
 */

// One span of the hierarchical timing trace pipeline/run.py records for a
// solve. `start_seconds` is relative to the start of the root span.
export type TraceNode = {
  name: string;
  start_seconds: number;
  duration_seconds: number;
  metadata?: Record<string, unknown>;
  children?: TraceNode[];
};

export const TraceNodeSchema: z.ZodType<TraceNode> = z.lazy(() =>
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
export const SolutionStopSchema = z.object({
  id: z.string(),
  name: z.string(),
  lat: z.number(),
  lon: z.number(),
  required: z.boolean(),
});
export type SolutionStop = z.infer<typeof SolutionStopSchema>;

// One GTFS route+direction a leg of the solution path travels on. The colors
// are GTFS route_color/route_text_color, without the "#", and may be empty.
export const SolutionRouteSchema = z.object({
  id: z.string(),
  name: z.string(),
  color: z.string(),
  text_color: z.string(),
});
export type SolutionRoute = z.infer<typeof SolutionRouteSchema>;

// One leg of the solution path. `route_direction_id` is null for a walk.
export const PathStepSchema = z.object({
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
export const SolutionPathSchema = z.object({
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
export const SolutionDataSchema = z
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
export type SolutionData = z.infer<typeof SolutionDataSchema>;

// One problem instance, joined with the spec and target stops it came from.
export const SolutionSchema = z.object({
  problem_instance_id: z.string(),
  problem_spec_id: z.string(),
  spec_title: z.string(),
  target_stops_id: z.string(),
  target_stops_title: z.string(),
  service_date: z.string(),
  gtfs_instance_id: z.string(),
  data: SolutionDataSchema,
});
export type Solution = z.infer<typeof SolutionSchema>;

// One GTFS fetch, i.e. one pipeline run, as the client sees it: the server
// reads `fetched_at` as a Date and sends it as an ISO string.
export const RunSchema = z.object({
  gtfs_instance_id: z.string(),
  fetched_at: z.string(),
});
export type Run = z.infer<typeof RunSchema>;
