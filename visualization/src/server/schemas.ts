import { z } from "zod";

// Zod schemas matching the C++ visualization schemas
export const StopSchema = z.object({
  stop_id: z.string(),
  stop_name: z.string(),
  lat: z.number(),
  lon: z.number(),
});

export const VizPathSchema = z.object({
  depart_time: z.number(),
  arrive_time: z.number(),
  duration_seconds: z.number(),
  is_flex: z.boolean(),
});

export const VizPathGroupSchema = z.object({
  origin_stop_id: z.string(),
  destination_stop_id: z.string(),
  paths: z.array(VizPathSchema),
});

export const VisualizationSchema = z.object({
  stops: z.array(StopSchema),
  path_groups: z.array(VizPathGroupSchema),
});

export type Stop = z.infer<typeof StopSchema>;
export type VizPath = z.infer<typeof VizPathSchema>;
export type VizPathGroup = z.infer<typeof VizPathGroupSchema>;
export type Visualization = z.infer<typeof VisualizationSchema>;
