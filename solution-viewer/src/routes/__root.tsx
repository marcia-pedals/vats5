import { createRootRoute, Outlet } from "@tanstack/react-router";
import { useEffect } from "react";

export const Route = createRootRoute({
  component: RootLayout,
});

export function usePageTitle(page: string) {
  useEffect(() => {
    document.title = __CHECKOUT_NAME__ ? `[${__CHECKOUT_NAME__}] ${page}` : page;
  }, [page]);
}

function RootLayout() {
  return (
    <div className="min-h-screen bg-tc-void text-tc-text antialiased">
      <Outlet />
    </div>
  );
}
