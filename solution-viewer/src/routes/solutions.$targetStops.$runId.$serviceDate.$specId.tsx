import { createFileRoute, Link, useNavigate } from "@tanstack/react-router";
import { useEffect } from "react";
import { trpc } from "../client/trpc";
import { SolutionPathSummary, SolutionPathView } from "../components/SolutionPath";
import type { Solution, TraceNode } from "../schemas";
import {
  buildGrid,
  CELL_ROUTE,
  type CellParams,
  cellOf,
  type Grid,
  LATEST_RUN,
  landingSolution,
  resolveRunId,
} from "../solutions";
import { usePageTitle } from "./__root";

// The literal, not CELL_ROUTE: the router plugin reads this path statically.
export const Route = createFileRoute("/solutions/$targetStops/$runId/$serviceDate/$specId")({
  component: CellPage,
});

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

function SolutionCellContents({ solution }: { solution: Solution }) {
  const { status, optimal_duration_seconds } = solution.data;

  if (status === "solved" && optimal_duration_seconds !== undefined) {
    return <span className="text-tc-text">{formatDuration(optimal_duration_seconds)}</span>;
  }

  // Anything that is not a solve -- timeout included -- is a failure, shown as
  // an icon; the details panel says what went wrong.
  return (
    <span className="inline-flex text-tc-red align-middle" role="img" aria-label={status}>
      <ErrorIcon />
    </span>
  );
}

function SolutionCell({
  solution,
  params,
  current,
}: {
  solution?: Solution;
  params: CellParams;
  current: boolean;
}) {
  if (!solution) {
    return <span className="text-tc-text-dim">·</span>;
  }
  return (
    <Link
      to={CELL_ROUTE}
      params={params}
      className={
        "block w-full rounded-panel px-2 py-0.5 text-right transition-colors " +
        (current ? "bg-tc-cyan-dim ring-1 ring-tc-cyan" : "hover:bg-tc-overlay")
      }
      aria-current={current ? "page" : undefined}
    >
      <SolutionCellContents solution={solution} />
    </Link>
  );
}

function GridTable({
  grid,
  targetStopsId,
  runId,
  currentInstanceId,
}: {
  grid: Grid;
  targetStopsId: string;
  runId: string;
  currentInstanceId?: string;
}) {
  return (
    // Which target stop set this is is already named by the navbar links, so
    // the table is all the card carries.
    <section className="panel max-h-full shrink-0 overflow-auto p-3">
      <table className="w-auto table-fixed font-mono text-sm border-collapse">
        <thead>
          <tr className="text-tc-text-dim text-xs uppercase tracking-wider">
            {/* The dates below label themselves, so this column needs no header. */}
            <th className="w-28 py-1 pr-4" />
            {grid.specs.map((spec) => (
              <th
                key={spec.id}
                className="w-24 text-right font-medium py-1 pl-4 normal-case whitespace-nowrap"
                title={spec.id}
              >
                {spec.title}
              </th>
            ))}
          </tr>
        </thead>
        <tbody>
          {grid.serviceDates.map((serviceDate) => {
            const { weekday, label } = formatServiceDate(serviceDate);
            return (
              <tr key={serviceDate} className="border-t border-tc-border">
                <td className="py-1.5 pr-4 whitespace-nowrap">
                  <span className="text-tc-text">{weekday}</span>{" "}
                  <span className="text-tc-text-muted tabular-nums">{label}</span>
                </td>
                {grid.specs.map((spec) => {
                  const solution = cellOf(grid, serviceDate, spec.id);
                  return (
                    <td
                      key={spec.id}
                      className="py-1.5 pl-4 text-right tabular-nums whitespace-nowrap"
                    >
                      <SolutionCell
                        solution={solution}
                        params={{
                          targetStops: targetStopsId,
                          runId,
                          serviceDate,
                          specId: spec.id,
                        }}
                        current={solution?.problem_instance_id === currentInstanceId}
                      />
                    </td>
                  );
                })}
              </tr>
            );
          })}
        </tbody>
      </table>
    </section>
  );
}

/** 3.14 -> "3.1s", 614.2 -> "10m14s". Wall-clock, not solution durations. */
function formatSeconds(seconds: number): string {
  if (seconds < 60) {
    return `${seconds.toFixed(1)}s`;
  }
  const minutes = Math.floor(seconds / 60);
  return `${minutes}m${String(Math.round(seconds % 60)).padStart(2, "0")}s`;
}

type TraceRow = { key: string; node: TraceNode; depth: number };

function flattenTrace(node: TraceNode, depth = 0, key = "0"): TraceRow[] {
  return [
    { key, node, depth },
    ...(node.children ?? []).flatMap((child, index) =>
      flattenTrace(child, depth + 1, `${key}.${index}`)
    ),
  ];
}

// Bar color by depth, so the nesting reads at a glance. Deeper than this
// repeats the last one.
const TRACE_DEPTH_COLORS = ["bg-tc-blue", "bg-tc-cyan", "bg-tc-magenta"];

// Chip color per value of a span's `solver` metadata, so that which solver ran
// for which iteration reads down the column rather than having to be read.
const SOLVER_CLASS: Record<string, string> = {
  brute: "bg-tc-cyan-dim text-tc-cyan",
  bnb: "bg-tc-blue-dim text-tc-blue",
};

/**
 * A chart of one solve: every span laid out against the root's span, so
 * gaps (time inside a span that nothing below accounts for) are visible.
 */
function TraceChart({ root }: { root: TraceNode }) {
  const rows = flattenTrace(root);
  // A zero-length root would put every bar at width 0; nothing to lay out.
  const total = root.duration_seconds;
  if (total <= 0) {
    return <p className="font-mono text-xs text-tc-text-muted">Trace has no duration.</p>;
  }

  // Rows are laid out one grid each, so the column widths have to be fixed for
  // them to line up; long span names truncate rather than shift the bars.
  return (
    <div className="space-y-1">
      <div className="grid grid-cols-[16rem_1fr_4rem] items-center gap-3 font-mono text-[10px] uppercase tracking-wider text-tc-text-dim">
        <span>span</span>
        <span className="flex justify-between">
          <span>0s</span>
          <span>{formatSeconds(total)}</span>
        </span>
        <span className="text-right normal-case tracking-normal">took</span>
      </div>

      {rows.map(({ key, node, depth }) => {
        const metadata = Object.entries(node.metadata ?? {});
        return (
          <div
            key={key}
            className="grid grid-cols-[16rem_1fr_4rem] items-center gap-3 rounded-panel py-0.5 font-mono text-xs hover:bg-tc-surface"
            title={`${node.name}\nstart ${node.start_seconds.toFixed(1)}s\nduration ${node.duration_seconds.toFixed(1)}s`}
          >
            <span
              className="truncate whitespace-nowrap text-tc-text"
              style={{ paddingLeft: `${depth * 0.75}rem` }}
            >
              {node.name}
              {metadata.map(([metaKey, value]) =>
                metaKey === "solver" ? (
                  // Which solver ran is the one piece of metadata worth
                  // spotting without reading, so it gets a chip of its own.
                  <span
                    key={metaKey}
                    className={`ml-1.5 rounded-sm px-1 ${
                      SOLVER_CLASS[String(value)] ?? "bg-tc-subtle text-tc-text-muted"
                    }`}
                  >
                    {String(value)}
                  </span>
                ) : (
                  <span key={metaKey} className="ml-1.5 text-tc-text-dim">
                    {metaKey.replace(/_/g, " ")} {String(value)}
                  </span>
                )
              )}
            </span>

            <span className="relative block h-2.5 rounded-sm bg-tc-subtle">
              <span
                className={`absolute top-0 h-full rounded-sm ${
                  TRACE_DEPTH_COLORS[Math.min(depth, TRACE_DEPTH_COLORS.length - 1)]
                }`}
                style={{
                  left: `${(node.start_seconds / total) * 100}%`,
                  width: `${(node.duration_seconds / total) * 100}%`,
                  minWidth: "2px",
                }}
              />
            </span>

            <span className="text-right tabular-nums text-tc-text-muted">
              {formatSeconds(node.duration_seconds)}
            </span>
          </div>
        );
      })}
    </div>
  );
}

function SolutionDetails({ solution }: { solution: Solution }) {
  // Everything named here is either laid out below or, like the optimal
  // duration -- which the summary in the title row already gives -- pulled out
  // so `rest` does not render it again as a generic metadata row.
  const { status, optimal_duration_seconds, trace, stops, routes, solution_path, ...rest } =
    solution.data;
  const { weekday, label } = formatServiceDate(solution.service_date);
  const solved = status === "solved";

  return (
    <section className="panel max-h-full min-w-0 flex-1 space-y-4 overflow-y-auto p-3">
      {/* A cell is always shown, so there is nothing to close back to. The
          path's one-line gist rides along here rather than taking a row of
          its own under the map. */}
      <header className="flex flex-wrap items-baseline justify-between gap-x-4 gap-y-1">
        <h2 className="whitespace-nowrap font-display text-base text-tc-text">
          {weekday} {label} <span className="text-tc-text-muted">· {solution.spec_title}</span>
        </h2>
        {solution_path && stops && <SolutionPathSummary stops={stops} path={solution_path} />}
      </header>

      {/* The path is what the card is for, so it runs straight on from the
          title rather than under a heading repeating what it obviously is. */}
      {solution_path && stops ? (
        <SolutionPathView stops={stops} routes={routes ?? []} path={solution_path} />
      ) : (
        <p className="font-mono text-xs text-tc-text-muted">
          {solved ? "No path recorded for this run." : "Only a solved instance has a path."}
        </p>
      )}

      <div className="space-y-2 border-t border-tc-border pt-3">
        <h3 className="font-mono text-xs uppercase tracking-wider text-tc-text-muted">Timing</h3>
        {trace ? (
          <TraceChart root={trace} />
        ) : (
          <p className="font-mono text-xs text-tc-text-muted">
            No timing trace recorded for this run.
          </p>
        )}
      </div>

      <div className="space-y-2 border-t border-tc-border pt-3">
        <h3 className="font-mono text-xs uppercase tracking-wider text-tc-text-muted">Metadata</h3>
        <dl className="grid grid-cols-[7rem_1fr] gap-x-3 gap-y-1 font-mono text-xs">
          <dt className="text-tc-text-dim">status</dt>
          <dd className={solved ? "text-tc-green" : "text-tc-red"}>{status.replace(/_/g, " ")}</dd>
          {Object.entries(rest).map(([key, value]) => (
            <div key={key} className="contents">
              <dt className="text-tc-text-dim">{key.replace(/_/g, " ")}</dt>
              <dd className="text-tc-text break-all">{JSON.stringify(value)}</dd>
            </div>
          ))}
          <dt className="text-tc-text-dim">logs</dt>
          <dd className="text-tc-text break-all">
            {solution.gtfs_instance_id}/service_{solution.service_date}/{solution.problem_spec_id}
          </dd>
        </dl>
      </div>
    </section>
  );
}

function CellPage() {
  const { targetStops: targetStopsId, runId: runParam, serviceDate, specId } = Route.useParams();
  usePageTitle(`Solutions ${specId} ${serviceDate}`);
  const navigate = useNavigate();

  // Shared with the layout above, which needs the same list for its steppers;
  // react-query hands both the one fetch.
  const runsQuery = trpc.listRuns.useQuery({ targetStopsId });
  const runId = resolveRunId(runParam, runsQuery.data ?? []);

  const solutionsQuery = trpc.listSolutions.useQuery(
    { gtfsInstanceId: runId ?? "", targetStopsId },
    { enabled: runId !== undefined }
  );
  const grid = buildGrid(solutionsQuery.data ?? []);
  const empty = solutionsQuery.data?.length === 0;
  const solution = cellOf(grid, serviceDate, specId);

  // /latest is a way in, not an address to stay at: as soon as it resolves it
  // is swapped for the run's own id, so the URL in the bar always names the run
  // actually on screen and keeps naming it once a newer run lands.
  const aliasedRunId = runParam === LATEST_RUN ? runId : undefined;
  useEffect(() => {
    if (aliasedRunId) {
      navigate({
        to: CELL_ROUTE,
        params: { targetStops: targetStopsId, runId: aliasedRunId, serviceDate, specId },
        replace: true,
      });
    }
  }, [aliasedRunId, targetStopsId, serviceDate, specId, navigate]);

  // The URL names the cell on show, so one this run has no solve for is
  // corrected -- to the nearest cell the run does have -- rather than quietly
  // showing something the address does not say. Held off while the alias above
  // is still resolving, so only one redirect is in flight.
  const landing = solution || aliasedRunId ? undefined : landingSolution(grid, serviceDate, specId);
  const landingDate = landing?.service_date;
  const landingSpec = landing?.problem_spec_id;
  useEffect(() => {
    if (landingDate && landingSpec) {
      navigate({
        to: CELL_ROUTE,
        params: {
          targetStops: targetStopsId,
          runId: runParam,
          serviceDate: landingDate,
          specId: landingSpec,
        },
        replace: true,
      });
    }
  }, [landingDate, landingSpec, runParam, targetStopsId, navigate]);

  const error = solutionsQuery.error;
  const pending = runsQuery.isPending || solutionsQuery.isPending;

  // Only the page's own content: the navbar and the shell around it belong to
  // the layout route, which stays mounted while this swaps.
  return (
    <>
      {pending && !error && (
        <p className="text-tc-text-muted font-mono text-sm animate-pulse">Loading...</p>
      )}

      {error && <p className="text-tc-red text-sm font-mono">ERR: {error.message}</p>}

      {empty && (
        <p className="text-tc-text-muted font-mono text-sm">
          Nothing solved for target stops {targetStopsId} in {runId}.
        </p>
      )}

      {/* The grid table sizes to its own columns, leaving the rest of the
          width for the solution details card. Each scrolls on its own. */}
      {!pending && !empty && (
        <div className="flex min-h-0 flex-1 items-start gap-3">
          <GridTable
            grid={grid}
            targetStopsId={targetStopsId}
            runId={runId ?? runParam}
            currentInstanceId={solution?.problem_instance_id}
          />
          {solution && <SolutionDetails solution={solution} />}
        </div>
      )}
    </>
  );
}
