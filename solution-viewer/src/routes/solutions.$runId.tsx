import { createFileRoute, useNavigate } from "@tanstack/react-router";
import { trpc } from "../client/trpc";
import { usePageTitle } from "./__root";

export const Route = createFileRoute("/solutions/$runId")({
  component: RunPage,
});

type Solution = {
  problem_instance_id: string;
  problem_spec_id: string;
  spec_title: string;
  target_stops_id: string;
  target_stops_title: string;
  service_date: string;
  gtfs_instance_id: string;
  data: {
    status: string;
    optimal_duration_seconds?: number;
    timeout_seconds?: number;
    returncode?: number | null;
  };
};

/** 21300 -> "5h55m", 2040 -> "34m". */
function formatDuration(seconds: number): string {
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.round((seconds % 3600) / 60);
  if (hours === 0) {
    return `${minutes}m`;
  }
  return `${hours}h${String(minutes).padStart(2, "0")}m`;
}

/** "20260810" -> { weekday: "Mon", label: "08-10" }. */
function formatServiceDate(serviceDate: string): { weekday: string; label: string } {
  const year = Number(serviceDate.slice(0, 4));
  const month = serviceDate.slice(4, 6);
  const day = serviceDate.slice(6, 8);
  const date = new Date(year, Number(month) - 1, Number(day));
  return {
    weekday: date.toLocaleDateString(undefined, { weekday: "short" }),
    label: `${month}-${day}`,
  };
}

/** One target stop set: service dates down, the specs using it across. */
type Group = {
  targetStopsId: string;
  targetStopsTitle: string;
  specs: { id: string; title: string }[];
  serviceDates: string[];
  cells: Map<string, Solution>;
};

function groupByTargetStops(solutions: Solution[]): Group[] {
  const groups = new Map<string, Group>();
  for (const solution of solutions) {
    let group = groups.get(solution.target_stops_id);
    if (!group) {
      group = {
        targetStopsId: solution.target_stops_id,
        targetStopsTitle: solution.target_stops_title,
        specs: [],
        serviceDates: [],
        cells: new Map(),
      };
      groups.set(solution.target_stops_id, group);
    }
    if (!group.specs.some((spec) => spec.id === solution.problem_spec_id)) {
      group.specs.push({ id: solution.problem_spec_id, title: solution.spec_title });
    }
    if (!group.serviceDates.includes(solution.service_date)) {
      group.serviceDates.push(solution.service_date);
    }
    group.cells.set(`${solution.problem_spec_id}|${solution.service_date}`, solution);
  }
  for (const group of groups.values()) {
    group.specs.sort((a, b) => a.id.localeCompare(b.id));
    group.serviceDates.sort();
  }
  return [...groups.values()];
}

function ErrorIcon() {
  return (
    <svg
      width="15"
      height="15"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <path d="m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3" />
      <path d="M12 9v4" />
      <path d="M12 17h.01" />
    </svg>
  );
}

function SolutionCell({ solution }: { solution?: Solution }) {
  if (!solution) {
    return <span className="text-tc-text-dim">·</span>;
  }
  const { status, optimal_duration_seconds } = solution.data;

  if (status === "solved" && optimal_duration_seconds !== undefined) {
    return (
      <span className="text-tc-text" title={`${optimal_duration_seconds} seconds`}>
        {formatDuration(optimal_duration_seconds)}
      </span>
    );
  }

  // Anything that is not a solve -- timeout included -- is a failure: an icon
  // with everything needed to go find the logs in its tooltip.
  const { returncode, ...rest } = solution.data;
  const details = [
    status.replace(/_/g, " "),
    returncode === undefined || returncode === null ? null : `exit code ${returncode}`,
    ...Object.entries(rest)
      .filter(([key]) => key !== "status")
      .map(([key, value]) => `${key}: ${JSON.stringify(value)}`),
    `logs: ${solution.gtfs_instance_id}/service_${solution.service_date}/${solution.problem_spec_id}`,
  ]
    .filter((line) => line !== null)
    .join("\n");

  return (
    <span
      className="inline-flex text-tc-red cursor-help align-middle"
      title={details}
      aria-label={details}
      role="img"
    >
      <ErrorIcon />
    </span>
  );
}

function GroupTable({ group }: { group: Group }) {
  return (
    <section className="panel space-y-3 overflow-x-auto">
      <header>
        <h2 className="font-display text-base text-tc-text">{group.targetStopsTitle}</h2>
      </header>

      {/* Fixed column widths, chosen to be wider than any cell: every group's
          table is then the same width, so the cards line up instead of each
          sizing to its own contents. Full width instead would fling the columns
          to opposite edges of the page and make them hard to compare. */}
      <table className="w-auto table-fixed font-mono text-sm border-collapse">
        <thead>
          <tr className="text-tc-text-dim text-xs uppercase tracking-wider">
            <th className="w-32 text-left font-medium py-1 pr-6 whitespace-nowrap">service date</th>
            {group.specs.map((spec) => (
              <th
                key={spec.id}
                className="w-28 text-right font-medium py-1 pl-6 normal-case whitespace-nowrap"
                title={spec.id}
              >
                {spec.title}
              </th>
            ))}
          </tr>
        </thead>
        <tbody>
          {group.serviceDates.map((serviceDate) => {
            const { weekday, label } = formatServiceDate(serviceDate);
            return (
              <tr key={serviceDate} className="border-t border-tc-border">
                <td className="py-1.5 pr-6 whitespace-nowrap">
                  <span className="text-tc-text">{weekday}</span>{" "}
                  <span className="text-tc-text-muted tabular-nums">{label}</span>
                </td>
                {group.specs.map((spec) => (
                  <td
                    key={spec.id}
                    className="py-1.5 pl-6 text-right tabular-nums whitespace-nowrap"
                  >
                    <SolutionCell solution={group.cells.get(`${spec.id}|${serviceDate}`)} />
                  </td>
                ))}
              </tr>
            );
          })}
        </tbody>
      </table>
    </section>
  );
}

function ChevronIcon({ direction }: { direction: "left" | "right" }) {
  return (
    <svg
      width="16"
      height="16"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <path d={direction === "left" ? "m15 18-6-6 6-6" : "m9 18 6-6-6-6"} />
    </svg>
  );
}

const STEP_BUTTON_CLASS =
  "rounded-panel border border-tc-border bg-tc-raised p-1.5 text-tc-text-muted " +
  "transition-colors hover:border-tc-border-bright hover:text-tc-text " +
  "disabled:cursor-not-allowed disabled:opacity-40 disabled:hover:border-tc-border " +
  "disabled:hover:text-tc-text-muted";

function RunBar({ runId }: { runId: string }) {
  const navigate = useNavigate();
  const runsQuery = trpc.listRuns.useQuery();
  const runs = runsQuery.data ?? [];
  const index = runs.findIndex((run) => run.gtfs_instance_id === runId);

  // Runs are newest first, so stepping to an older run moves down the list.
  const olderRun = index < 0 ? undefined : runs[index + 1];
  const newerRun = index < 0 ? undefined : runs[index - 1];

  const goTo = (targetRunId: string) => {
    navigate({ to: "/solutions/$runId", params: { runId: targetRunId } });
  };

  return (
    <nav className="flex items-center gap-2">
      <button
        type="button"
        className={STEP_BUTTON_CLASS}
        disabled={!olderRun}
        onClick={() => olderRun && goTo(olderRun.gtfs_instance_id)}
        title={olderRun ? `Older: ${olderRun.gtfs_instance_id}` : "No older run"}
        aria-label="Older run"
      >
        <ChevronIcon direction="left" />
      </button>

      <select
        className="rounded-panel border border-tc-border bg-tc-raised px-3 py-1.5 font-mono text-xs text-tc-text"
        value={runId}
        onChange={(event) => goTo(event.target.value)}
        aria-label="Run"
      >
        {index < 0 && (
          <option value={runId} disabled>
            {runId} (unknown)
          </option>
        )}
        {runs.map((run) => (
          <option key={run.gtfs_instance_id} value={run.gtfs_instance_id}>
            {run.gtfs_instance_id} · {new Date(run.fetched_at).toLocaleString()}
          </option>
        ))}
      </select>

      <button
        type="button"
        className={STEP_BUTTON_CLASS}
        disabled={!newerRun}
        onClick={() => newerRun && goTo(newerRun.gtfs_instance_id)}
        title={newerRun ? `Newer: ${newerRun.gtfs_instance_id}` : "No newer run"}
        aria-label="Newer run"
      >
        <ChevronIcon direction="right" />
      </button>
    </nav>
  );
}

function RunPage() {
  const { runId } = Route.useParams();
  usePageTitle(`Solutions ${runId}`);
  const solutionsQuery = trpc.listSolutions.useQuery({ gtfsInstanceId: runId });
  const groups = groupByTargetStops(solutionsQuery.data ?? []);

  return (
    <div className="min-h-screen px-6 py-10 flex justify-center">
      <div className="w-full max-w-6xl space-y-6">
        <header className="border-b border-tc-border pb-2">
          <h1 className="font-mono text-sm tracking-widest uppercase text-tc-text-muted">
            Solutions
          </h1>
        </header>

        <RunBar runId={runId} />

        {solutionsQuery.isPending && (
          <p className="text-tc-text-muted font-mono text-sm animate-pulse">Loading...</p>
        )}

        {solutionsQuery.error && (
          <p className="text-tc-red text-sm font-mono">ERR: {solutionsQuery.error.message}</p>
        )}

        {solutionsQuery.data && groups.length === 0 && (
          <p className="text-tc-text-muted font-mono text-sm">No problem instances for {runId}.</p>
        )}

        {/* Group cards size to their own table, leaving the right-hand side
            free for a solution details card. */}
        <div className="flex items-start gap-6">
          <div className="flex flex-col items-start gap-6">
            {groups.map((group) => (
              <GroupTable key={group.targetStopsId} group={group} />
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}
