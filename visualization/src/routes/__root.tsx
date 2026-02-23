import { createRootRoute, Outlet } from "@tanstack/react-router";
import { useEffect } from "react";

export const Route = createRootRoute({
  component: RootLayout,
});

export function useCheckoutTitle(page: string) {
  useEffect(() => {
    document.title = `[${__CHECKOUT_NAME__}] ${page}`;
  }, [page]);
}

function RootLayout() {
  useCheckoutTitle("Visualization");
  return (
    <div className="min-h-screen bg-tc-void text-tc-text antialiased">
      <Outlet />
    </div>
  );
}
