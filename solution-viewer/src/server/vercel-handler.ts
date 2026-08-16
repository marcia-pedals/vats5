import { fetchRequestHandler } from "@trpc/server/adapters/fetch";
import { appRouter } from "./trpc";

// The tRPC router behind the deployed function. Locally the same router is
// served by server.ts (Express); here it runs on Vercel's Node.js runtime
// using the Web-standard Request/Response signature.
//
// `npm run build:server` bundles this file, and everything it imports from
// src/, into the single file that api/trpc/[trpc].ts re-exports.
export default function handler(request: Request): Promise<Response> {
  // vercel.json rewrites /trpc/* to this function, so requests can arrive
  // under either prefix; the handler needs the one actually in the URL.
  const { pathname } = new URL(request.url);
  const endpoint = pathname.startsWith("/api/trpc") ? "/api/trpc" : "/trpc";

  return fetchRequestHandler({
    endpoint,
    req: request,
    router: appRouter,
    createContext: () => ({}),
  });
}
