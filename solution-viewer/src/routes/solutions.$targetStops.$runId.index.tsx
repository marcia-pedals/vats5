import { createFileRoute, useNavigate } from "@tanstack/react-router";
import { useEffect } from "react";
import { trpc } from "../client/trpc";
import { buildGrid, CELL_ROUTE, firstSolution, resolveRunId } from "../solutions";
import { usePageTitle } from "./__root";

export const Route = createFileRoute("/solutions/$targetStops/$runId/")({
  component: RunIndexPage,
});

/**
 * /solutions/<target stops>/<run> is not a page of its own -- a page always
 * shows one cell, so this redirects to the run's first one. Also where the
 * `latest` alias is resolved for a bare run URL.
 */
function RunIndexPage() {
  const { targetStops: targetStopsId, runId: runParam } = Route.useParams();
  usePageTitle(`Solutions ${targetStopsId}`);
  const navigate = useNavigate();

  const runsQuery = trpc.listRuns.useQuery({ targetStopsId });
  const runId = resolveRunId(runParam, runsQuery.data ?? []);
  const solutionsQuery = trpc.listSolutions.useQuery(
    { gtfsInstanceId: runId ?? "", targetStopsId },
    { enabled: runId !== undefined }
  );
  const first = firstSolution(buildGrid(solutionsQuery.data ?? []));
  const serviceDate = first?.service_date;
  const specId = first?.problem_spec_id;

  useEffect(() => {
    if (runId && serviceDate && specId) {
      navigate({
        // The resolved id, not the param: /latest is a way in, and what it
        // redirects to names the run it landed on.
        to: CELL_ROUTE,
        params: { targetStops: targetStopsId, runId, serviceDate, specId },
        replace: true,
      });
    }
  }, [runId, serviceDate, specId, targetStopsId, navigate]);

  // Only the page's own content; the navbar and shell are the layout route's,
  // so they stay put while this resolves and redirects.
  return (
    <div className="font-mono text-sm">
      {solutionsQuery.error && <p className="text-tc-red">ERR: {solutionsQuery.error.message}</p>}
      {solutionsQuery.data?.length === 0 && (
        <p className="text-tc-text-muted">
          Nothing solved for target stops {targetStopsId} in {runId}.
        </p>
      )}
      {!solutionsQuery.error && !solutionsQuery.data && runsQuery.data?.length !== 0 && (
        <p className="text-tc-text-muted animate-pulse">Loading...</p>
      )}
    </div>
  );
}
