import { createFileRoute, useNavigate } from "@tanstack/react-router";
import { useEffect } from "react";
import { trpc } from "../client/trpc";
import { usePageTitle } from "./__root";

export const Route = createFileRoute("/solutions/")({
  component: SolutionsIndexPage,
});

/** /solutions is not a page of its own -- it redirects to the newest run. */
function SolutionsIndexPage() {
  usePageTitle("Solutions");
  const navigate = useNavigate();
  const runsQuery = trpc.listRuns.useQuery();
  const latestRunId = runsQuery.data?.[0]?.gtfs_instance_id;

  useEffect(() => {
    if (latestRunId) {
      navigate({
        to: "/solutions/$runId",
        params: { runId: latestRunId },
        replace: true,
      });
    }
  }, [latestRunId, navigate]);

  return (
    <div className="min-h-screen px-6 py-10 flex justify-center">
      <div className="w-full max-w-6xl font-mono text-sm">
        {runsQuery.error && <p className="text-tc-red">ERR: {runsQuery.error.message}</p>}
        {runsQuery.data && runsQuery.data.length === 0 && (
          <p className="text-tc-text-muted">No runs yet — run pipeline/run.py.</p>
        )}
        {!runsQuery.error && runsQuery.data?.length !== 0 && (
          <p className="text-tc-text-muted animate-pulse">Loading...</p>
        )}
      </div>
    </div>
  );
}
