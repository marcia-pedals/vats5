import * as fs from "node:fs/promises";
import { homedir } from "node:os";
import * as path from "node:path";
import Database from "better-sqlite3";
import { z } from "zod";

const PROBLEMS_DIR = path.join(homedir(), "vats5-problems");

// LRU pool of open read-only database connections.
const DB_POOL_MAX = 4;
const dbPool = new Map<string, Database.Database>();

function getDb(name: string): Database.Database {
  const existing = dbPool.get(name);
  if (existing) {
    // Move to end (most recently used)
    dbPool.delete(name);
    dbPool.set(name, existing);
    return existing;
  }

  const filePath = path.join(PROBLEMS_DIR, `${name}-viz.sqlite`);
  const resolvedPath = path.resolve(filePath);
  const resolvedProblemsDir = path.resolve(PROBLEMS_DIR);
  if (!resolvedPath.startsWith(resolvedProblemsDir)) {
    throw new Error("Invalid file path");
  }

  const db = new Database(filePath, { readonly: true });

  // Evict oldest if at capacity
  if (dbPool.size >= DB_POOL_MAX) {
    const oldest = dbPool.keys().next().value!;
    dbPool.get(oldest)!.close();
    dbPool.delete(oldest);
  }

  dbPool.set(name, db);
  return db;
}

// --- Schemas ---

const StopSchema = z.object({
  stop_id: z.number(),
  gtfs_stop_id: z.string(),
  stop_name: z.string(),
  lat: z.number(),
  lon: z.number(),
  required: z.number(),
});
export type Stop = z.infer<typeof StopSchema>;

const VizPathRowSchema = z.object({
  path_id: z.number(),
  depart_time: z.number(),
  arrive_time: z.number(),
  is_flex: z.number(),
});

export interface VizPath {
  path_id: number;
  depart_time: number;
  arrive_time: number;
  is_flex: boolean;
}

const PathStepRowSchema = z.object({
  origin_stop_name: z.string(),
  destination_stop_name: z.string(),
  depart_time: z.number(),
  arrive_time: z.number(),
  is_flex: z.number(),
});

export interface PathStep {
  origin_stop_name: string;
  destination_stop_name: string;
  depart_time: number;
  arrive_time: number;
  is_flex: boolean;
}

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
  const db = getDb(name);
  const rows = db
    .prepare("SELECT stop_id, gtfs_stop_id, stop_name, lat, lon, required FROM stops")
    .all();
  return z.array(StopSchema).parse(rows);
}

export function getPaths(name: string, origin: number, destination: number): VizPath[] {
  const db = getDb(name);
  const rows = db
    .prepare(
      "SELECT path_id, depart_time, arrive_time, is_flex FROM paths WHERE origin_stop_id = ? AND destination_stop_id = ?"
    )
    .all(origin, destination);
  const parsed = z.array(VizPathRowSchema).parse(rows);
  return parsed.map((r) => ({
    path_id: r.path_id,
    depart_time: r.depart_time,
    arrive_time: r.arrive_time,
    is_flex: r.is_flex === 1,
  }));
}

export function getPathSteps(name: string, pathId: number): PathStep[] {
  const db = getDb(name);
  const rows = db
    .prepare(
      `SELECT o.stop_name AS origin_stop_name, d.stop_name AS destination_stop_name,
              ps.depart_time, ps.arrive_time, ps.is_flex
       FROM paths_steps ps
       JOIN stops o ON o.stop_id = ps.origin_stop_id
       JOIN stops d ON d.stop_id = ps.destination_stop_id
       WHERE ps.path_id = ?`
    )
    .all(pathId);
  const parsed = z.array(PathStepRowSchema).parse(rows);
  return parsed.map((r) => ({
    origin_stop_name: r.origin_stop_name,
    destination_stop_name: r.destination_stop_name,
    depart_time: r.depart_time,
    arrive_time: r.arrive_time,
    is_flex: r.is_flex === 1,
  }));
}
