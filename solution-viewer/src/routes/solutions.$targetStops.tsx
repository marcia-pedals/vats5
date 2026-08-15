import { createFileRoute, Link, Outlet, useParams } from "@tanstack/react-router";
import { type ReactNode, useId } from "react";
import { trpc } from "../client/trpc";
import type { Run } from "../schemas";
import { CELL_ROUTE, type CellParams, LATEST_RUN, resolveRunId } from "../solutions";

/**
 * The chrome around every solutions page: which target stop set is shown, and
 * which of its runs.
 *
 * A layout route rather than part of the page so that it survives navigation.
 * Switching target stops passes through an index route that has no cell to show
 * yet; with the bar inside the page it would unmount and flash away, and with
 * it out here only the outlet below swaps.
 */
export const Route = createFileRoute("/solutions/$targetStops")({
  component: TargetStopsLayout,
});

function StepIcon({ shape }: { shape: "left" | "right" | "newest" }) {
  return (
    <svg
      width="14"
      height="14"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      {shape === "left" && <path d="m15 18-6-6 6-6" />}
      {shape === "right" && <path d="m9 18 6-6-6-6" />}
      {/* Doubled, for the jump past every run in between. */}
      {shape === "newest" && <path d="m6 17 5-5-5-5" />}
      {shape === "newest" && <path d="m13 17 5-5-5-5" />}
    </svg>
  );
}

const STEP_CLASS = "rounded-panel border border-tc-border bg-tc-raised p-1 block";

/**
 * A hover tooltip for whatever it wraps. Its own, rather than the browser's
 * `title`, so that it shows at once instead of after the native delay and is
 * styled like the rest of the app.
 */
function Tooltip({ text, children }: { text: string; children: ReactNode }) {
  const id = useId();
  return (
    <span className="group relative inline-flex">
      <span aria-describedby={id}>{children}</span>
      <span
        id={id}
        role="tooltip"
        className="pointer-events-none absolute left-1/2 top-full z-20 mt-1.5 -translate-x-1/2 whitespace-nowrap rounded-panel border border-tc-border bg-tc-raised px-2 py-1 font-mono text-[10px] text-tc-text-muted opacity-0 shadow-sm transition-opacity duration-100 group-hover:opacity-100"
      >
        {text}
      </span>
    </span>
  );
}

/** One step of the run stepper: a link, or a disabled control with nowhere to go. */
function RunStep({
  shape,
  label,
  emptyTitle,
  run,
  params,
}: {
  shape: "left" | "right" | "newest";
  label: string;
  emptyTitle: string;
  run?: Run;
  params?: CellParams;
}) {
  // No run that way -- or no cell to carry across yet -- means there is nothing
  // to link to, and a disabled control says so where an inert span would just
  // look like a dead link.
  if (!run || !params) {
    return (
      <button
        type="button"
        disabled
        className={`${STEP_CLASS} cursor-not-allowed text-tc-text-muted opacity-40`}
        title={emptyTitle}
        aria-label={label}
      >
        <StepIcon shape={shape} />
      </button>
    );
  }
  return (
    <Link
      to={CELL_ROUTE}
      params={params}
      className={`${STEP_CLASS} text-tc-text-muted transition-colors hover:border-tc-border-bright hover:text-tc-text`}
      title={`${label}: ${run.gtfs_instance_id}`}
      aria-label={label}
    >
      <StepIcon shape={shape} />
    </Link>
  );
}

function TargetStopsLayout() {
  const { targetStops: targetStopsId } = Route.useParams();

  // The cell below, when there is one. Read loosely because the index routes
  // under here have no service date or spec yet.
  const { runId: runParam, serviceDate, specId } = useParams({ strict: false });

  const targetStopsQuery = trpc.listTargetStops.useQuery();
  // Runs are scoped to the target stop set, so the steppers only ever offer
  // runs that have something to show for it.
  const runsQuery = trpc.listRuns.useQuery({ targetStopsId });
  const runs = runsQuery.data ?? [];

  const resolved = runParam ? resolveRunId(runParam, runs) : undefined;
  const index = runs.findIndex((run) => run.gtfs_instance_id === resolved);
  const current = index < 0 ? undefined : runs[index];

  // Runs are newest first, so stepping to an older run moves down the list, and
  // the newest run is the head of it -- offered only when not already there.
  const olderRun = index < 0 ? undefined : runs[index + 1];
  const newerRun = index < 0 ? undefined : runs[index - 1];
  const newestRun = index === 0 ? undefined : runs[0];

  // Stepping runs holds the cell in place, so it needs the full address; always
  // the run's own id, never /latest, since a URL is worth sharing only if it
  // still names the same run tomorrow. The page corrects the address if the run
  // stepped to has no such date or spec.
  const runParams = (run?: Run): CellParams | undefined =>
    run && serviceDate && specId
      ? { targetStops: targetStopsId, runId: run.gtfs_instance_id, serviceDate, specId }
      : undefined;

  return (
    <div className="flex h-full flex-col px-3">
      <nav className="flex shrink-0 flex-wrap items-center gap-2 border-b border-tc-border py-1.5">
        {/* A segmented control rather than bare words: the sunken track is what
            says these are several things to pick between, without shouting. */}
        <ul className="flex flex-wrap items-center gap-0.5 rounded-panel border border-tc-border bg-tc-surface p-0.5">
          {(targetStopsQuery.data ?? []).map((targetStops) => {
            const showing = targetStops.target_stops_id === targetStopsId;
            return (
              <li key={targetStops.target_stops_id}>
                {/* Each set has its own runs, so switching lands on that set's
                    newest one rather than carrying this set's run across. */}
                <Link
                  to="/solutions/$targetStops/$runId"
                  params={{ targetStops: targetStops.target_stops_id, runId: LATEST_RUN }}
                  className={
                    "block rounded-sm px-2 py-0.5 font-mono text-xs transition-colors " +
                    (showing
                      ? "bg-tc-raised text-tc-cyan ring-1 ring-tc-border"
                      : "text-tc-text-muted hover:bg-tc-overlay hover:text-tc-text")
                  }
                  aria-current={showing ? "page" : undefined}
                >
                  {targetStops.title}
                </Link>
              </li>
            );
          })}
        </ul>

        <span className="mx-1 h-5 w-px shrink-0 bg-tc-border" aria-hidden="true" />

        <RunStep
          shape="left"
          label="Older run"
          emptyTitle="No older run"
          run={olderRun}
          params={runParams(olderRun)}
        />

        {/* The run is a readout, not a control: the steppers change it. The bare
            timestamp is all the bar carries; what it means is a hover away
            rather than spelled out in a label. */}
        <Tooltip text="When the schedule data was fetched">
          {/* Matched to the steppers' own height and centered in it, so the text
              sits on their axis rather than on its own baseline. */}
          <span className="flex h-6 items-center font-mono text-xs text-tc-text">
            {current ? (
              new Date(current.fetched_at).toLocaleString()
            ) : (
              <span className="text-tc-text-dim">
                {runsQuery.isPending ? "Loading run..." : "Unknown run"}
              </span>
            )}
          </span>
        </Tooltip>

        <RunStep
          shape="right"
          label="Newer run"
          emptyTitle="No newer run"
          run={newerRun}
          params={runParams(newerRun)}
        />
        <RunStep
          shape="newest"
          label="Newest run"
          emptyTitle="Already on the newest run"
          run={newestRun}
          params={runParams(newestRun)}
        />
      </nav>

      {/* min-h-0 against flex's default floor, so that the panels below take
          the overflow instead of growing the page past the viewport. */}
      <div className="flex min-h-0 flex-1 flex-col py-3">
        {runsQuery.error && (
          <p className="text-tc-red text-sm font-mono">ERR: {runsQuery.error.message}</p>
        )}
        {runsQuery.data?.length === 0 && (
          <p className="text-tc-text-muted font-mono text-sm">
            No runs for target stops {targetStopsId}.
          </p>
        )}
        <Outlet />
      </div>
    </div>
  );
}
