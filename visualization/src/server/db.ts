import * as fs from "node:fs/promises";
import { homedir } from "node:os";
import * as path from "node:path";
import Database from "better-sqlite3";
import { z } from "zod";

const PROBLEMS_DIR = path.join(homedir(), "vats5-problems");

function openDb(name: string): Database.Database {
  const filePath = path.join(PROBLEMS_DIR, `${name}-viz.sqlite`);
  const resolvedPath = path.resolve(filePath);
  const resolvedProblemsDir = path.resolve(PROBLEMS_DIR);
  if (!resolvedPath.startsWith(resolvedProblemsDir)) {
    throw new Error("Invalid file path");
  }
  return new Database(filePath, { readonly: true });
}

// --- Schemas ---

const StopSchema = z.object({
  stop_id: z.string(),
  stop_name: z.string(),
  lat: z.number(),
  lon: z.number(),
  stop_type: z.enum(["required", "in_problem_state", "original"]),
});
export type Stop = z.infer<typeof StopSchema>;

const VizPathRowSchema = z.object({
  path_id: z.number(),
  depart_time: z.number(),
  arrive_time: z.number(),
  is_flex: z.number(),
});
export type VizPath = z.infer<typeof VizPathRowSchema>;

const PathStepRowSchema = z.object({
  origin_stop_id: z.string(),
  destination_stop_id: z.string(),
  depart_time: z.number(),
  arrive_time: z.number(),
  is_flex: z.number(),
  route_name: z.string(),
});
export type PathStep = z.infer<typeof PathStepRowSchema>;

// --- Queries ---

export async function listVisualizations(): Promise<{ filename: string; name: string }[]> {
  try {
    const files = await fs.readdir(PROBLEMS_DIR);
    return files
      .filter((file) => file.endsWith("-viz.sqlite"))
      .map((file) => ({
        filename: file,
        name: file.replace("-viz.sqlite", ""),
      }));
  } catch (error) {
    console.error("Error listing visualizations:", error);
    throw new Error("Failed to list visualizations");
  }
}

export function getStops(name: string): Stop[] {
  const db = openDb(name);
  try {
    const rows = db.prepare("SELECT stop_id, stop_name, lat, lon, stop_type FROM stops").all();
    return z.array(StopSchema).parse(rows);
  } finally {
    db.close();
  }
}

export function getPaths(name: string, origin: string, destination: string): VizPath[] {
  const db = openDb(name);
  try {
    const rows = db
      .prepare(
        "SELECT path_id, depart_time, arrive_time, is_flex FROM paths WHERE origin_stop_id = ? AND destination_stop_id = ?"
      )
      .all(origin, destination);
    return z.array(VizPathRowSchema).parse(rows);
  } finally {
    db.close();
  }
}

export function getPathSteps(name: string, pathId: number): PathStep[] {
  const db = openDb(name);
  try {
    const rows = db
      .prepare(
        "SELECT origin_stop_id, destination_stop_id, depart_time, arrive_time, is_flex, route_name FROM paths_steps WHERE path_id = ?"
      )
      .all(pathId);
    return z.array(PathStepRowSchema).parse(rows);
  } finally {
    db.close();
  }
}
