import { createFileRoute, useNavigate } from "@tanstack/react-router";
import { useState } from "react";
import { trpc } from "../client/trpc";
import { SolutionPathView } from "../components/SolutionPath";
import type { Solution, TraceNode } from "../schemas";
import { usePageTitle } from "./__root";

export const Route = createFileRoute("/solutions/$runId")({
  component: RunPage,
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
  selected,
  onSelect,
}: {
  solution?: Solution;
  selected: boolean;
  onSelect: (solution: Solution) => void;
}) {
  if (!solution) {
    return <span className="text-tc-text-dim">·</span>;
  }
  return (
    <button
      type="button"
      className={
        "w-full rounded-panel px-2 py-0.5 text-right transition-colors " +
        (selected ? "bg-tc-cyan-dim ring-1 ring-tc-cyan" : "hover:bg-tc-overlay cursor-pointer")
      }
      aria-pressed={selected}
      onClick={() => onSelect(solution)}
    >
      <SolutionCellContents solution={solution} />
    </button>
  );
}

function GroupTable({
  group,
  selectedId,
  onSelect,
}: {
  group: Group;
  selectedId?: string;
  onSelect: (solution: Solution) => void;
}) {
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
                {group.specs.map((spec) => {
                  const solution = group.cells.get(`${spec.id}|${serviceDate}`);
                  return (
                    <td
                      key={spec.id}
                      className="py-1.5 pl-6 text-right tabular-nums whitespace-nowrap"
                    >
                      <SolutionCell
                        solution={solution}
                        selected={solution?.problem_instance_id === selectedId}
                        onSelect={onSelect}
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

function SolutionDetails({ solution, onClose }: { solution: Solution; onClose: () => void }) {
  const { status, optimal_duration_seconds, trace, stops, routes, solution_path, ...rest } =
    solution.data;
  const { weekday, label } = formatServiceDate(solution.service_date);
  const solved = status === "solved";

  return (
    <section className="panel flex-1 min-w-0 space-y-4 sticky top-10">
      <header className="flex items-start justify-between gap-4">
        <div>
          <h2 className="font-display text-base text-tc-text">
            {solution.spec_title}{" "}
            <span className="text-tc-text-muted">
              · {weekday} {label}
            </span>
          </h2>
          <p className="font-mono text-xs text-tc-text-dim">{solution.problem_spec_id}</p>
        </div>
        <button
          type="button"
          className="rounded-panel px-2 font-mono text-sm text-tc-text-dim hover:text-tc-text cursor-pointer"
          onClick={onClose}
          aria-label="Close details"
        >
          ✕
        </button>
      </header>

      <dl className="grid grid-cols-[7rem_1fr] gap-x-3 gap-y-1 font-mono text-xs">
        <dt className="text-tc-text-dim">status</dt>
        <dd className={solved ? "text-tc-green" : "text-tc-red"}>{status.replace(/_/g, " ")}</dd>
        {optimal_duration_seconds !== undefined && (
          <>
            <dt className="text-tc-text-dim">optimal</dt>
            <dd className="text-tc-text">
              {formatDuration(optimal_duration_seconds)} ({optimal_duration_seconds}s)
            </dd>
          </>
        )}
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

      <div className="space-y-2 border-t border-tc-border pt-3">
        <h3 className="font-mono text-xs uppercase tracking-wider text-tc-text-muted">Path</h3>
        {solution_path && stops ? (
          <SolutionPathView stops={stops} routes={routes ?? []} path={solution_path} />
        ) : (
          <p className="font-mono text-xs text-tc-text-muted">
            {solved ? "No path recorded for this run." : "Only a solved instance has a path."}
          </p>
        )}
      </div>

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
    </section>
  );
}

function RunPage() {
  const { runId } = Route.useParams();
  usePageTitle(`Solutions ${runId}`);
  const solutionsQuery = trpc.listSolutions.useQuery({ gtfsInstanceId: runId });
  const solutions = solutionsQuery.data ?? [];
  const groups = groupByTargetStops(solutions);

  // Held by id rather than by value so that switching runs drops a selection
  // that the new run has no cell for.
  const [selectedId, setSelectedId] = useState<string>();
  const selected = solutions.find((solution) => solution.problem_instance_id === selectedId);

  return (
    <div className="min-h-screen px-6 py-10 flex justify-center">
      <div className="w-full max-w-7xl space-y-6">
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
              <GroupTable
                key={group.targetStopsId}
                group={group}
                selectedId={selectedId}
                onSelect={(solution) => setSelectedId(solution.problem_instance_id)}
              />
            ))}
          </div>
          {selected && (
            <SolutionDetails solution={selected} onClose={() => setSelectedId(undefined)} />
          )}
        </div>
      </div>
    </div>
  );
}
