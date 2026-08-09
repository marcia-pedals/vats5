import { httpBatchLink } from "@trpc/client";
import { createTRPCReact } from "@trpc/react-query";
import type { AppRouter } from "../server/trpc";

export const trpc = createTRPCReact<AppRouter>();

export const trpcClient = trpc.createClient({
  links: [
    httpBatchLink({
      // Locally vite proxies /trpc to the Express server; on Vercel a rewrite
      // sends it to the serverless function.
      url: "/trpc",
    }),
  ],
});
