import { initTRPC } from "@trpc/server";
import { z } from "zod";

const t = initTRPC.create();

export const appRouter = t.router({
  greeting: t.procedure.input(z.object({ name: z.string() })).query(({ input }) => {
    return { message: `Hello, ${input.name}!` };
  }),

  getItems: t.procedure.query(() => {
    return [
      { id: 1, name: "Item 1" },
      { id: 2, name: "Item 2" },
      { id: 3, name: "Item 3" },
    ];
  }),
});

export type AppRouter = typeof appRouter;
