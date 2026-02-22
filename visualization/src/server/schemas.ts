import { z } from "zod";

// Zod schemas matching the C++ visualization schemas
export const StopSchema = z.object({
  stop_id: z.string(),
  stop_name: z.string(),
  lat: z.number(),
  lon: z.number(),
});

export const VisualizationSchema = z.object({
  stops: z.array(StopSchema),
});

export type Stop = z.infer<typeof StopSchema>;
export type Visualization = z.infer<typeof VisualizationSchema>;
