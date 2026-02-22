import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import {
  createRootRoute,
  createRoute,
  createRouter,
  RouterProvider,
  Outlet,
} from "@tanstack/react-router";
import React from "react";
import ReactDOM from "react-dom/client";
import { IndexPage, VizPage } from "./client/App";
import { trpc, trpcClient } from "./client/trpc";

const queryClient = new QueryClient();

const rootRoute = createRootRoute({
  component: Outlet,
});

const indexRoute = createRoute({
  getParentRoute: () => rootRoute,
  path: "/",
  component: IndexPage,
});

const vizRoute = createRoute({
  getParentRoute: () => rootRoute,
  path: "/viz/$name",
  component: VizPage,
});

const routeTree = rootRoute.addChildren([indexRoute, vizRoute]);
const router = createRouter({ routeTree });

declare module "@tanstack/react-router" {
  interface Register {
    router: typeof router;
  }
}

const rootElement = document.getElementById("root");
if (!rootElement) {
  throw new Error("Root element not found");
}

ReactDOM.createRoot(rootElement).render(
  <React.StrictMode>
    <trpc.Provider client={trpcClient} queryClient={queryClient}>
      <QueryClientProvider client={queryClient}>
        <RouterProvider router={router} />
      </QueryClientProvider>
    </trpc.Provider>
  </React.StrictMode>
);
