import { existsSync } from "node:fs";
import * as path from "node:path";
import { fileURLToPath } from "node:url";
import dotenv from "dotenv";

// Local development only: load DATABASE_URL (and friends) from the app's
// .env.local / .env. In deployed environments (Vercel) the variables are
// injected by the platform and no file exists.
const appDir = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../..");

for (const file of [".env.local", ".env"]) {
  const filePath = path.join(appDir, file);
  if (existsSync(filePath)) {
    dotenv.config({ path: filePath });
  }
}
