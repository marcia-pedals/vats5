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

export type Stop = z.infer<typeof StopSchema>;
export type VizPath = z.infer<typeof VizPathSchema>;
