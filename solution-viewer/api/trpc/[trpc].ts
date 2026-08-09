import { fetchRequestHandler } from "@trpc/server/adapters/fetch";
import { appRouter } from "../../src/server/trpc";

// Vercel serverless function. Locally the same router is served by
// server.ts (Express); here it runs on the platform's Node.js runtime using
// the Web-standard Request/Response signature.
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
