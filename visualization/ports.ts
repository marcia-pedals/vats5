import { createHash } from "node:crypto";
import path from "node:path";

// Derive a stable port offset from the checkout directory path so that
// multiple checkouts of the same repo can run dev servers simultaneously
// without port conflicts.  Each checkout always lands on the same ports.
const checkoutDir = path.resolve(process.cwd(), "..");
const hash = createHash("md5").update(checkoutDir).digest();
const offset = hash.readUInt16BE(0) % 1000;

export const VITE_PORT = 3000 + offset;
export const TRPC_PORT = 4000 + offset;
export const CHECKOUT_NAME = path.basename(checkoutDir);
