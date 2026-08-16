import { fetchRequestHandler } from "@trpc/server/adapters/fetch";
// Deployed imports carry the file extension: Vercel compiles each traced .ts
// file to .js in place without rewriting import specifiers, so extensionless
// paths are unresolvable in the deployed ES module. `.ts` here keeps the file
// traceable at build time; `rewriteRelativeImportExtensions` (tsconfig.json)
// turns it into `.js` on emit.
import { appRouter } from "../../src/server/trpc.ts";

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
