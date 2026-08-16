import { fetchRequestHandler } from "@trpc/server/adapters/fetch";
import { appRouter } from "./trpc";

function handler(request: Request): Promise<Response> {
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

// The tRPC router behind the deployed function. Locally the same router is
// served by server.ts (Express); here it runs on Vercel's Node.js runtime.
//
// Exported as `fetch` rather than as a bare function: Vercel picks the calling
// convention from the export shape, and a bare default export means the Node
// (request, response) signature, whose `url` is a path like "/trpc/x" rather
// than something `new URL()` accepts. A `fetch` export gets the Web-standard
// Request this handler expects, and covers every HTTP method.
//
// `npm run build:server` bundles this file, and everything it imports from
// src/, into the single file that api/trpc/[trpc].ts re-exports.
export default { fetch: handler };
