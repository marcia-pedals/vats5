import { createFileRoute, redirect } from "@tanstack/react-router";

// Solutions are the whole app; / is just a way in.
export const Route = createFileRoute("/")({
  beforeLoad: () => {
    throw redirect({ to: "/solutions", replace: true });
  },
});
