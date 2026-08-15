import { createFileRoute, redirect } from "@tanstack/react-router";
import { LATEST_RUN } from "../solutions";

// /solutions/<target stops> is not a page of its own -- a page always shows one
// run. Which run is newest depends on data, so this hands off to the /latest
// alias rather than resolving it here.
export const Route = createFileRoute("/solutions/$targetStops/")({
  beforeLoad: ({ params }) => {
    throw redirect({
      to: "/solutions/$targetStops/$runId",
      params: { targetStops: params.targetStops, runId: LATEST_RUN },
      replace: true,
    });
  },
});
