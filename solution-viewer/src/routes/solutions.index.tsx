import { createFileRoute, useNavigate } from "@tanstack/react-router";
import { useEffect } from "react";
import { trpc } from "../client/trpc";
import { usePageTitle } from "./__root";

export const Route = createFileRoute("/solutions/")({
  component: SolutionsIndexPage,
});

/**
 * /solutions is not a page of its own -- the target stop set is the outer axis,
 * so this redirects to the first one, which in turn takes its own newest run.
 */
function SolutionsIndexPage() {
  usePageTitle("Solutions");
  const navigate = useNavigate();

  const targetStopsQuery = trpc.listTargetStops.useQuery();
  const first = targetStopsQuery.data?.[0]?.target_stops_id;

  useEffect(() => {
    if (first) {
      navigate({
        to: "/solutions/$targetStops",
        params: { targetStops: first },
        replace: true,
      });
    }
  }, [first, navigate]);

  return (
    <div className="px-3 py-3 font-mono text-sm">
      {targetStopsQuery.error && (
        <p className="text-tc-red">ERR: {targetStopsQuery.error.message}</p>
      )}
      {targetStopsQuery.data?.length === 0 && (
        <p className="text-tc-text-muted">No solutions yet — run pipeline/run.py.</p>
      )}
      {!targetStopsQuery.error && !targetStopsQuery.data && (
        <p className="text-tc-text-muted animate-pulse">Loading...</p>
      )}
    </div>
  );
}
