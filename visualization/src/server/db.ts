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
  stop_id: z.string(),
  stop_name: z.string(),
  lat: z.number(),
  lon: z.number(),
  required: z.number(),
});
export type Stop = z.infer<typeof StopSchema>;

const VizPathRowSchema = z.object({
  depart_time: z.number(),
  arrive_time: z.number(),
  duration_seconds: z.number(),
  is_flex: z.number(),
});

export interface VizPath {
  depart_time: number;
  arrive_time: number;
  duration_seconds: number;
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
  const rows = db.prepare("SELECT stop_id, stop_name, lat, lon, required FROM stops").all();
  return z.array(StopSchema).parse(rows);
}

export function getPaths(name: string, origin: string, destination: string): VizPath[] {
  const db = getDb(name);
  const rows = db
    .prepare(
      "SELECT depart_time, arrive_time, duration_seconds, is_flex FROM paths WHERE origin_stop_id = ? AND destination_stop_id = ?"
    )
    .all(origin, destination);
  const parsed = z.array(VizPathRowSchema).parse(rows);
  return parsed.map((r) => ({
    depart_time: r.depart_time,
    arrive_time: r.arrive_time,
    duration_seconds: r.duration_seconds,
    is_flex: r.is_flex === 1,
  }));
}
