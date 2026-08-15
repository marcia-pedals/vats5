import type { Run, Solution } from "./schemas";

/**
 * How the solution pages address a cell, shared by the pages themselves and by
 * the index routes that redirect into them.
 *
 * The target stop set is the outer axis: it is picked first, and everything
 * below -- which runs exist, which specs and service dates they cover -- is
 * scoped to it.
 */

/** The route every cell link points at; the page for one problem instance. */
export const CELL_ROUTE = "/solutions/$targetStops/$runId/$serviceDate/$specId";

/** The params that name one cell; every link into the grid builds a set of these. */
export type CellParams = {
  targetStops: string;
  runId: string;
  serviceDate: string;
  specId: string;
};

/**
 * The `$runId` that means "whichever run is newest for this target stop set".
 * A way in rather than an address to stay at: pages resolve it and rewrite the
 * URL to the run's own id.
 */
export const LATEST_RUN = "latest";

/** The run a `$runId` param names; undefined until the run list has loaded. */
export function resolveRunId(runParam: string, runs: Run[]): string | undefined {
  return runParam === LATEST_RUN ? runs[0]?.gtfs_instance_id : runParam;
}

/** One target stop set in one run: service dates down, the specs using it across. */
export type Grid = {
  specs: { id: string; title: string }[];
  serviceDates: string[];
  cells: Map<string, Solution>;
};

export function buildGrid(solutions: Solution[]): Grid {
  const grid: Grid = { specs: [], serviceDates: [], cells: new Map() };
  for (const solution of solutions) {
    if (!grid.specs.some((spec) => spec.id === solution.problem_spec_id)) {
      grid.specs.push({ id: solution.problem_spec_id, title: solution.spec_title });
    }
    if (!grid.serviceDates.includes(solution.service_date)) {
      grid.serviceDates.push(solution.service_date);
    }
    grid.cells.set(`${solution.problem_spec_id}|${solution.service_date}`, solution);
  }
  grid.specs.sort((a, b) => a.id.localeCompare(b.id));
  grid.serviceDates.sort();
  return grid;
}

/** The grid's solve for one spec on one service date, if the run has it. */
export function cellOf(grid: Grid, serviceDate: string, specId: string): Solution | undefined {
  return grid.cells.get(`${specId}|${serviceDate}`);
}

/**
 * The grid's first cell in reading order: service dates down, specs across.
 * Undefined for an empty grid, which is what a run with nothing solved for
 * this target stop set looks like.
 */
export function firstSolution(grid: Grid): Solution | undefined {
  for (const serviceDate of grid.serviceDates) {
    for (const spec of grid.specs) {
      const solution = cellOf(grid, serviceDate, spec.id);
      if (solution) {
        return solution;
      }
    }
  }
  return undefined;
}

/**
 * Where a URL naming a cell this grid does not have should land instead: the
 * same service date under whichever spec the grid does cover, failing that its
 * first cell.
 */
export function landingSolution(
  grid: Grid,
  serviceDate: string,
  specId: string
): Solution | undefined {
  const exact = cellOf(grid, serviceDate, specId);
  if (exact) {
    return exact;
  }
  for (const spec of grid.specs) {
    const sameDate = cellOf(grid, serviceDate, spec.id);
    if (sameDate) {
      return sameDate;
    }
  }
  return firstSolution(grid);
}
