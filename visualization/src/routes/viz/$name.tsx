import { skipToken } from "@tanstack/react-query";
import { createFileRoute, Link } from "@tanstack/react-router";
import { useGesture } from "@use-gesture/react";
import { ArrowUpDown, ChevronLeft, ChevronRight, X } from "lucide-react";
import React, { useCallback, useMemo, useRef, useState } from "react";
import { trpc } from "../../client/trpc";
import type {
  PartialSolutionData,
  PathStep,
  Stop,
  VizPath,
} from "../../server/db";
import { useCheckoutTitle } from "../__root";

export const Route = createFileRoute("/viz/$name")({
  component: VizPage,
});

const DOT_R = 5;
const LABEL_FONT_SIZE = 10; // constant screen-px
const LABEL_CHAR_W = 6.2; // approx monospace char width at 10px
const LABEL_PAD = 2; // collision padding (screen px)
const LABEL_MIN_NEIGHBOR = 15; // screen-px: suppress label if nearest neighbor closer than this

interface Transform {
  x: number;
  y: number;
  scale: number;
}

const DEFAULT_TRANSFORM: Transform = { x: 0, y: 0, scale: 1 };

function formatTime(seconds: number): string {
  const neg = seconds < 0;
  const abs = Math.abs(seconds);
  const h = Math.floor(abs / 3600);
  const m = Math.floor((abs % 3600) / 60);
  const s = abs % 60;
  const pad = (n: number) => n.toString().padStart(2, "0");
  return `${neg ? "-" : ""}${pad(h)}:${pad(m)}:${pad(s)}`;
}

function formatDuration(seconds: number): string {
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  const s = seconds % 60;
  const mm = String(m).padStart(2, "0");
  const ss = String(s).padStart(2, "0");
  if (h > 0) return `${h}:${mm}:${ss}`;
  return `${mm}:${ss}`;
}

function StopDropdown({
  label,
  value,
  stops,
  onChange,
  onClear,
}: {
  label: string;
  value: string | null;
  stops: Stop[];
  onChange: (stopId: string) => void;
  onClear: () => void;
}) {
  return (
    <div className="flex items-center gap-1.5">
      <span className="text-[10px] font-mono text-tc-text-dim w-8 shrink-0">
        {label}
      </span>
      <select
        value={value ?? ""}
        onChange={(e) => {
          if (e.target.value) onChange(e.target.value);
        }}
        className="flex-1 min-w-0 text-xs font-mono bg-tc-raised border border-tc-border rounded px-1.5 py-1 text-tc-text truncate cursor-pointer"
      >
        <option value="" disabled>
          — select station —
        </option>
        {stops.map((s) => (
          <option key={s.stop_id} value={s.stop_id}>
            {s.stop_name}
          </option>
        ))}
      </select>
      <button
        type="button"
        onClick={onClear}
        disabled={value === null}
        className="w-5 h-5 flex items-center justify-center rounded text-tc-text-dim border border-tc-border bg-transparent hover:border-tc-red/50 hover:text-tc-red transition-colors cursor-pointer disabled:opacity-0 disabled:cursor-default shrink-0"
        aria-label={`Clear ${label.toLowerCase()}`}
      >
        <X size={10} />
      </button>
    </div>
  );
}

function PathSteps({
  steps,
  path,
  stopNames,
  startCol = 1,
}: {
  steps: PathStep[];
  path: VizPath;
  stopNames: Map<string, string>;
  startCol?: number;
}) {
  return (
    <>
      {steps.map((step, i) => (
        <React.Fragment key={`${step.depart_time}-${step.arrive_time}-${i}`}>
          {step.route_name && (
            <tr className="bg-tc-raised/50">
              {startCol > 0 && <td />}
              <td className="w-[70px] px-2 py-1 text-[11px] text-tc-text-dim whitespace-nowrap">
                {path.is_flex ? "" : formatTime(step.depart_time)}
              </td>
              <td className="px-2 py-1 text-[11px] text-tc-cyan" colSpan={2}>
                {step.is_flex ? "Walk" : step.route_name}
              </td>
            </tr>
          )}
          <tr className="bg-tc-raised/50 border-b border-tc-border/30">
            {startCol > 0 && <td />}
            <td className="w-[70px] px-2 py-1 text-[11px] text-tc-text-dim whitespace-nowrap">
              {path.is_flex ? "" : formatTime(step.arrive_time)}
            </td>
            <td
              className="px-2 py-1 text-[11px] text-tc-text-muted"
              colSpan={2}
            >
              {stopNames.get(step.destination_stop_id) ??
                `stop ${step.destination_stop_id}`}
            </td>
          </tr>
        </React.Fragment>
      ))}
    </>
  );
}

function PathRow({
  name,
  path,
  stopNames,
}: {
  name: string;
  path: VizPath;
  stopNames: Map<string, string>;
}) {
  const [expanded, setExpanded] = useState(false);
  const stepsQuery = trpc.getPathSteps.useQuery(
    expanded ? { name, pathId: path.path_id } : skipToken,
  );

  return (
    <>
      <tr
        className="border-b border-tc-border/50 hover:bg-tc-cyan-dim/40 transition-colors cursor-pointer"
        onClick={() => setExpanded((prev) => !prev)}
      >
        <td className="pl-1.5 pr-0 py-1.5 text-tc-text-dim">
          <ChevronRight
            size={12}
            className={`transition-transform ${expanded ? "rotate-90" : ""}`}
          />
        </td>
        <td className="w-[70px] px-2 py-1.5 text-tc-text whitespace-nowrap">
          {path.is_flex ? "**:**:**" : formatTime(path.depart_time)}
        </td>
        <td className="w-[70px] px-2 py-1.5 text-tc-text whitespace-nowrap">
          {path.is_flex ? "**:**:**" : formatTime(path.arrive_time)}
        </td>
        <td className="px-2 py-1.5 text-right text-tc-text-muted">
          {formatDuration(path.arrive_time - path.depart_time)}
        </td>
      </tr>
      {expanded && stepsQuery.isPending && (
        <tr>
          <td
            colSpan={4}
            className="px-2 py-1.5 text-[10px] font-mono text-tc-text-dim"
          >
            Loading steps...
          </td>
        </tr>
      )}
      {expanded && stepsQuery.data && (
        <PathSteps steps={stepsQuery.data} path={path} stopNames={stopNames} />
      )}
    </>
  );
}

function OverlayPathPanel({
  stopNames,
  partialData,
}: {
  stopNames: Map<string, string>;
  partialData: PartialSolutionData;
}) {
  const pathSteps = partialData.best_path.steps;
  const totalDuration = partialData.best_path.duration;

  // Exclude first and last steps
  const middleSteps = pathSteps.slice(1, -1);

  // Create a pseudo-path for the PathSteps component
  const pseudoPath: VizPath = {
    path_id: 0,
    depart_time: pathSteps[0]?.depart_time ?? 0,
    arrive_time: pathSteps[pathSteps.length - 1]?.arrive_time ?? 0,
    is_flex: 0,
  };

  return (
    <div className="absolute top-[104px] bottom-0 left-0 z-20 w-[340px] flex flex-col panel-surface m-3 overflow-hidden">
      {/* Header */}
      <div className="flex items-center justify-between px-3 py-2.5 border-b border-tc-border shrink-0">
        <span className="text-xs font-mono text-tc-text">Overlay Path</span>
        <span className="text-[10px] font-mono text-tc-text-dim">
          {formatDuration(totalDuration)} total
        </span>
      </div>

      {/* Path steps */}
      <div className="overflow-y-auto flex-1">
        <table className="w-full text-[11px] font-mono border-collapse">
          <tbody>
            <PathSteps
              steps={middleSteps}
              path={pseudoPath}
              stopNames={stopNames}
              startCol={0}
            />
          </tbody>
        </table>
      </div>

      {/* Summary */}
      <div className="px-3 py-2 border-t border-tc-border shrink-0">
        <span className="text-[10px] font-mono text-tc-text-dim">
          {middleSteps.length} segment{middleSteps.length !== 1 ? "s" : ""} ·{" "}
          {partialData.leaves.length} leaves
        </span>
      </div>
    </div>
  );
}

function StationPanel({
  name,
  stops,
  stopNames,
  origin,
  destination,
  onOriginChange,
  onDestChange,
  onOriginClear,
  onDestClear,
  onSwap,
}: {
  name: string;
  stops: Stop[];
  stopNames: Map<string, string>;
  origin: string | null;
  destination: string | null;
  onOriginChange: (stopId: string) => void;
  onDestChange: (stopId: string) => void;
  onOriginClear: () => void;
  onDestClear: () => void;
  onSwap: () => void;
}) {
  const pathsQuery = trpc.getPaths.useQuery(
    origin !== null && destination !== null
      ? { name, origin, destination }
      : skipToken,
  );
  const paths: VizPath[] | null = pathsQuery.data ?? null;

  return (
    <div className="absolute top-0 bottom-0 right-0 z-20 w-[340px] flex flex-col panel-surface m-3 overflow-hidden">
      {/* Station selectors */}
      <div className="flex items-center gap-2 px-3 py-2.5 border-b border-tc-border shrink-0">
        <button
          type="button"
          onClick={onSwap}
          disabled={origin === null && destination === null}
          className="w-5 h-5 flex items-center justify-center rounded text-tc-text-dim border border-tc-border bg-transparent hover:border-tc-cyan/50 hover:text-tc-cyan transition-colors cursor-pointer disabled:opacity-30 disabled:cursor-default shrink-0 self-center"
          aria-label="Swap origin and destination"
        >
          <ArrowUpDown size={10} />
        </button>
        <div className="flex flex-col gap-1.5 flex-1 min-w-0">
          <StopDropdown
            label="From"
            value={origin}
            stops={stops}
            onChange={onOriginChange}
            onClear={onOriginClear}
          />
          <StopDropdown
            label="To"
            value={destination}
            stops={stops}
            onChange={onDestChange}
            onClear={onDestClear}
          />
        </div>
      </div>

      {/* Path table */}
      {paths && paths.length > 0 && (
        <>
          <div className="flex items-center justify-between px-3 py-1.5 border-b border-tc-border shrink-0">
            <span className="text-[10px] font-mono text-tc-text-dim">
              {paths.length} path{paths.length !== 1 ? "s" : ""}
            </span>
          </div>
          <div className="overflow-y-auto flex-1">
            <table className="w-full text-[11px] font-mono border-collapse">
              <thead className="sticky top-0 bg-tc-surface">
                <tr className="text-tc-text-dim border-b border-tc-border">
                  <th className="w-5 px-0 py-1.5" />
                  <th className="w-[70px] text-left px-2 py-1.5 font-normal whitespace-nowrap">
                    Depart
                  </th>
                  <th className="w-[70px] text-left px-2 py-1.5 font-normal whitespace-nowrap">
                    Arrive
                  </th>
                  <th className="text-right px-2 py-1.5 font-normal">
                    Duration
                  </th>
                </tr>
              </thead>
              <tbody>
                {paths.map((p) => (
                  <PathRow
                    key={p.path_id}
                    name={name}
                    path={p}
                    stopNames={stopNames}
                  />
                ))}
              </tbody>
            </table>
          </div>
        </>
      )}

      {/* No paths message */}
      {origin !== null &&
        destination !== null &&
        paths !== null &&
        paths.length === 0 && (
          <div className="px-3 py-3 text-xs font-mono text-tc-text-dim">
            No paths found between these stops
          </div>
        )}

      {/* Empty state */}
      {(origin === null || destination === null) && (
        <div className="px-3 py-3 text-xs font-mono text-tc-text-dim">
          {origin === null
            ? "Click a station on the map to set origin"
            : "Click a station on the map to set destination"}
        </div>
      )}
    </div>
  );
}

// Stop dot colors — CSS variable values for SVG fill/stroke.
const STOP_COLORS = {
  // Non-required (in_problem_state) stops
  nonRequired: {
    fill: "var(--color-tc-text-dim)",
    stroke: "var(--color-tc-text-muted)",
  },
  // Default required stop
  required: { fill: "var(--color-tc-cyan)", stroke: "var(--color-tc-blue)" },
  // Unvisited required stop (partial solution active, stop not on path)
  unvisited: { fill: "var(--color-tc-red)", stroke: "var(--color-tc-red)" },
  // Leaf stop
  leaf: { fill: "var(--color-tc-amber)", stroke: "var(--color-tc-amber)" },
  // Selection ring
  ring: "var(--color-tc-blue)",
} as const;

function StopDot({
  stop,
  scale,
  isSelected,
  isRequired,
  isUnvisited,
  isLeaf,
  onClick,
}: {
  stop: { stop_id: string; cx: number; cy: number };
  scale: number;
  isSelected: boolean;
  isRequired: boolean;
  isUnvisited: boolean;
  isLeaf: boolean;
  onClick: (stopId: string) => void;
}) {
  if (!isRequired) {
    return (
      <circle
        cx={stop.cx}
        cy={stop.cy}
        r={(DOT_R - 1.5) / scale}
        fill={STOP_COLORS.nonRequired.fill}
        stroke={STOP_COLORS.nonRequired.stroke}
        strokeWidth={1 / scale}
        opacity={0.5}
        style={{ pointerEvents: "none" }}
      />
    );
  }

  const colors = (() => {
    if (isUnvisited) {
      return STOP_COLORS.unvisited;
    }
    if (isLeaf) {
      return STOP_COLORS.leaf;
    }
    return STOP_COLORS.required;
  })();

  const ringColor = STOP_COLORS.ring;

  return (
    // biome-ignore lint/a11y/noStaticElementInteractions: SVG <g> cannot be a <button>
    <g
      tabIndex={0}
      onClick={(e) => {
        e.stopPropagation();
        onClick(stop.stop_id);
      }}
      onKeyDown={(e) => {
        if (e.key === "Enter" || e.key === " ") onClick(stop.stop_id);
      }}
      style={{ cursor: "pointer", pointerEvents: "all" }}
    >
      {isSelected && (
        <circle
          cx={stop.cx}
          cy={stop.cy}
          r={(DOT_R + 4) / scale}
          fill="none"
          stroke={ringColor}
          strokeWidth={2 / scale}
          opacity={0.6}
        />
      )}
      <circle
        cx={stop.cx}
        cy={stop.cy}
        r={DOT_R / scale}
        fill={colors.fill}
        stroke={colors.stroke}
        strokeWidth={1 / scale}
        opacity={0.85}
      />
    </g>
  );
}

function VizPage() {
  const { name } = Route.useParams();
  useCheckoutTitle(name);

  const stopsQuery = trpc.getStops.useQuery({ name });
  const containerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<HTMLDivElement>(null);
  const [transform, setTransform] = useState<Transform>(DEFAULT_TRANSFORM);
  const [containerSize, setContainerSize] = useState<{
    w: number;
    h: number;
  } | null>(null);
  const [origin, setOrigin] = useState<string | null>(null);
  const [destination, setDestination] = useState<string | null>(null);
  const [selectedRun, setSelectedRun] = useState<string | null>(null);
  const [selectedIteration, setSelectedIteration] = useState(0);

  const runsQuery = trpc.getPartialSolutionRuns.useQuery(
    { name },
    { refetchInterval: 1000 },
  );
  const partialQuery = trpc.getPartialSolution.useQuery(
    selectedRun !== null
      ? { name, runTimestamp: selectedRun, iteration: selectedIteration }
      : skipToken,
    { refetchInterval: 1000 },
  );
  const selectedRunData = runsQuery.data?.find(
    (r) => r.run_timestamp === selectedRun,
  );
  const maxIteration = selectedRunData?.max_iteration ?? 0;

  const containerCallbackRef = useCallback((node: HTMLDivElement | null) => {
    containerRef.current = node;
    if (node) {
      setContainerSize({ w: node.clientWidth, h: node.clientHeight });
    }
  }, []);

  const svgStops = useMemo(() => {
    if (!stopsQuery.data?.length || !containerSize) return null;

    const stops = stopsQuery.data.filter((s) => s.stop_type !== "original");
    const lats = stops.map((s) => s.lat);
    const lons = stops.map((s) => s.lon);
    const minLat = Math.min(...lats);
    const maxLat = Math.max(...lats);
    const minLon = Math.min(...lons);
    const maxLon = Math.max(...lons);
    const latRange = maxLat - minLat || 1;
    const lonRange = maxLon - minLon || 1;

    const w = containerSize.w;
    const h = containerSize.h;
    const pad = 50;

    const midLat = (minLat + maxLat) / 2;
    const cosLat = Math.cos((midLat * Math.PI) / 180);
    const dataW = lonRange * cosLat;
    const dataH = latRange;
    const scalePerPixel = Math.min(
      (w - 2 * pad) / dataW,
      (h - 2 * pad) / dataH,
    );

    const drawW = dataW * scalePerPixel;
    const drawH = dataH * scalePerPixel;
    const offsetX = pad + (w - 2 * pad - drawW) / 2;
    const offsetY = pad + (h - 2 * pad - drawH) / 2;

    return stops.map((stop) => ({
      ...stop,
      cx: offsetX + (((stop.lon - minLon) * cosLat) / dataW) * drawW,
      cy: offsetY + ((maxLat - stop.lat) / dataH) * drawH,
    }));
  }, [stopsQuery.data, containerSize]);

  const handleStopClick = useCallback(
    (stopId: string) => {
      // Fill first blank slot; if both filled, change destination
      if (origin === null) {
        setOrigin(stopId);
      } else {
        setDestination(stopId);
      }
    },
    [origin],
  );

  // Greedy label placement: right or left, suppress in dense clusters
  const labelPlacements = useMemo(() => {
    if (!svgStops) return null;

    const scale = transform.scale;
    const gap = 4; // screen-px gap from dot edge
    const lh = LABEL_FONT_SIZE;
    const minNbrSq = LABEL_MIN_NEIGHBOR * LABEL_MIN_NEIGHBOR;

    // Relative screen positions (translation doesn't affect collisions)
    const sp = svgStops.map((s) => ({ x: s.cx * scale, y: s.cy * scale }));

    // Min neighbor distance (squared, screen space) for each stop
    const nbrDistSq = sp.map((p, i) => {
      let m = Infinity;
      for (let j = 0; j < sp.length; j++) {
        if (i === j) continue;
        const dx = p.x - sp[j].x,
          dy = p.y - sp[j].y;
        m = Math.min(m, dx * dx + dy * dy);
      }
      return m;
    });

    // Place most-isolated labels first (they anchor the layout)
    const order = sp
      .map((_, i) => i)
      .sort((a, b) => nbrDistSq[b] - nbrDistSq[a]);

    const boxes: [number, number, number, number][] = []; // placed bboxes
    const out: { dataX: number; dataY: number; visible: boolean }[] = new Array(
      svgStops.length,
    );

    for (const i of order) {
      // Suppress label entirely if too close to any neighbor
      if (nbrDistSq[i] < minNbrSq) {
        out[i] = { dataX: 0, dataY: 0, visible: false };
        continue;
      }

      const stop = svgStops[i];
      const sx = sp[i].x,
        sy = sp[i].y;
      const lw = LABEL_CHAR_W * stop.stop_name.length;

      // 2 candidates: right-center, left-center (vertically centered on dot)
      const cands: [number, number, number, number][] = [
        [sx + DOT_R + gap, sy - lh / 2, sx + DOT_R + gap + lw, sy + lh / 2],
        [sx - DOT_R - gap - lw, sy - lh / 2, sx - DOT_R - gap, sy + lh / 2],
      ];

      let placed = false;
      for (const [bx1, by1, bx2, by2] of cands) {
        const px1 = bx1 - LABEL_PAD,
          py1 = by1 - LABEL_PAD;
        const px2 = bx2 + LABEL_PAD,
          py2 = by2 + LABEL_PAD;

        // Check overlap with already-placed labels
        let hit = false;
        for (const [ox1, oy1, ox2, oy2] of boxes) {
          if (px1 < ox2 && px2 > ox1 && py1 < oy2 && py2 > oy1) {
            hit = true;
            break;
          }
        }
        // Check overlap with any dot
        if (!hit) {
          for (let j = 0; j < sp.length; j++) {
            if (j === i) continue;
            const cx = Math.max(px1, Math.min(sp[j].x, px2));
            const cy = Math.max(py1, Math.min(sp[j].y, py2));
            if ((sp[j].x - cx) ** 2 + (sp[j].y - cy) ** 2 < (DOT_R + 1) ** 2) {
              hit = true;
              break;
            }
          }
        }

        if (!hit) {
          out[i] = {
            dataX: bx1 / scale,
            dataY: (by1 + 0.75 * lh) / scale,
            visible: true,
          };
          boxes.push([px1, py1, px2, py2]);
          placed = true;
          break;
        }
      }

      if (!placed) out[i] = { dataX: 0, dataY: 0, visible: false };
    }
    return out;
  }, [svgStops, transform.scale]);

  useGesture(
    {
      onDrag: ({ delta: [dx, dy] }) => {
        setTransform((t) => ({ ...t, x: t.x + dx, y: t.y + dy }));
      },
      onWheel: ({ event }) => {
        event.preventDefault();
        const we = event as WheelEvent;
        const map = mapRef.current;
        if (!map) return;
        const rect = map.getBoundingClientRect();
        const px = we.clientX - rect.left;
        const py = we.clientY - rect.top;

        setTransform((t) => {
          // Dampen mouse wheel (large deltaY) but leave trackpad (small deltaY) alone
          const dy = Math.abs(we.deltaY) > 50 ? we.deltaY * 0.3 : we.deltaY;
          const factor = 1.005 ** -dy;
          const newScale = Math.min(Math.max(t.scale * factor, 0.1), 100);
          return {
            x: px - (px - t.x) * (newScale / t.scale),
            y: py - (py - t.y) * (newScale / t.scale),
            scale: newScale,
          };
        });
      },
    },
    {
      target: mapRef,
      drag: { filterTaps: true },
      eventOptions: { passive: false },
    },
  );

  const resetView = useCallback(() => {
    setTransform(DEFAULT_TRANSFORM);
    if (containerRef.current) {
      setContainerSize({
        w: containerRef.current.clientWidth,
        h: containerRef.current.clientHeight,
      });
    }
  }, []);

  const stopNames = useMemo(() => {
    const m = new Map<string, string>();
    if (stopsQuery.data) {
      for (const s of stopsQuery.data) m.set(s.stop_id, s.stop_name);
    }
    return m;
  }, [stopsQuery.data]);

  const selectedStops = useMemo(() => {
    const s = new Set<string>();
    if (origin !== null) s.add(origin);
    if (destination !== null) s.add(destination);
    return s;
  }, [origin, destination]);

  const isSelected = (stopId: string) => selectedStops.has(stopId);

  const stopCoords = useMemo(() => {
    if (!svgStops) return new Map<string, { cx: number; cy: number }>();
    const m = new Map<string, { cx: number; cy: number }>();
    for (const s of svgStops) {
      m.set(s.stop_id, { cx: s.cx, cy: s.cy });
    }
    return m;
  }, [svgStops]);

  const leafSet = useMemo(() => {
    if (!partialQuery.data) return new Set<string>();
    return new Set(partialQuery.data.leaves);
  }, [partialQuery.data]);

  const visitedStopSet = useMemo(() => {
    if (!partialQuery.data) return new Set<string>();
    console.log(partialQuery.data);
    const s = new Set<string>();
    for (const step of partialQuery.data.best_path.original_steps) {
      s.add(step.origin_stop_id);
      s.add(step.destination_stop_id);
    }
    return s;
  }, [partialQuery.data]);

  const pathArrows = useMemo(() => {
    if (!partialQuery.data || !stopCoords.size) return [];
    const steps = partialQuery.data.best_path.original_steps;
    const arrows: { x1: number; y1: number; x2: number; y2: number }[] = [];
    for (const step of steps) {
      const orig = stopCoords.get(step.origin_stop_id);
      const dest = stopCoords.get(step.destination_stop_id);
      if (!orig || !dest) continue;
      arrows.push({ x1: orig.cx, y1: orig.cy, x2: dest.cx, y2: dest.cy });
    }
    return arrows;
  }, [partialQuery.data, stopCoords]);

  return (
    <div
      ref={containerCallbackRef}
      className="relative w-screen h-screen overflow-hidden bg-tc-void select-none"
    >
      {/* Control bar */}
      <div className="absolute top-3 left-3 z-10 flex items-center gap-1.5 panel-surface py-1.5 px-2.5">
        <Link
          to="/"
          className="inline-flex items-center px-2.5 py-1 rounded text-xs font-mono text-tc-text-muted border border-tc-border hover:border-tc-cyan/50 hover:text-tc-cyan transition-colors no-underline"
        >
          &larr; back
        </Link>
        <button
          type="button"
          onClick={resetView}
          className="px-2.5 py-1 rounded text-xs font-mono text-tc-text-muted border border-tc-border bg-transparent hover:border-tc-cyan/50 hover:text-tc-cyan transition-colors cursor-pointer"
        >
          Reset view
        </button>
        <span className="text-xs font-mono text-tc-text-dim ml-2">{name}</span>
      </div>

      {/* Partial solutions selector */}
      {runsQuery.data && runsQuery.data.length > 0 && (
        <div className="absolute top-[52px] left-3 z-10 flex items-center gap-1.5 panel-surface py-1.5 px-2.5">
          <select
            value={selectedRun ?? ""}
            onChange={(e) => {
              const v = e.target.value || null;
              setSelectedRun(v);
              setSelectedIteration(0);
            }}
            className="text-xs font-mono bg-tc-raised border border-tc-border rounded px-1.5 py-1 text-tc-text cursor-pointer"
          >
            <option value="">— no overlay —</option>
            {runsQuery.data.map((run) => (
              <option key={run.run_timestamp} value={run.run_timestamp}>
                {run.run_timestamp}
              </option>
            ))}
          </select>
          {selectedRun !== null && (
            <>
              <button
                type="button"
                onClick={() => setSelectedIteration((i) => Math.max(0, i - 1))}
                disabled={selectedIteration === 0}
                className="w-5 h-5 flex items-center justify-center rounded text-tc-text-dim border border-tc-border bg-transparent hover:border-tc-cyan/50 hover:text-tc-cyan transition-colors cursor-pointer disabled:opacity-30 disabled:cursor-default"
              >
                <ChevronLeft size={12} />
              </button>
              <span className="text-xs font-mono text-tc-text-muted min-w-[48px] text-center">
                iter {selectedIteration}
              </span>
              <button
                type="button"
                onClick={() =>
                  setSelectedIteration((i) => Math.min(maxIteration, i + 1))
                }
                disabled={selectedIteration === maxIteration}
                className="w-5 h-5 flex items-center justify-center rounded text-tc-text-dim border border-tc-border bg-transparent hover:border-tc-cyan/50 hover:text-tc-cyan transition-colors cursor-pointer disabled:opacity-30 disabled:cursor-default"
              >
                <ChevronRight size={12} />
              </button>
              {partialQuery.data && (
                <span className="text-[10px] font-mono text-tc-text-dim">
                  {partialQuery.data.leaves.length} leaves &middot;{" "}
                  {formatDuration(partialQuery.data.best_path.duration)}
                </span>
              )}
            </>
          )}
        </div>
      )}

      {/* Map area — gesture target is scoped here so the panel is unaffected */}
      <div ref={mapRef} className="absolute inset-0 touch-none cursor-grab">
        {/* Loading */}
        {stopsQuery.isPending && (
          <div className="absolute inset-0 flex items-center justify-center">
            <span className="text-tc-text-muted font-mono text-sm animate-pulse">
              Loading feed...
            </span>
          </div>
        )}

        {/* Error */}
        {stopsQuery.error && (
          <div className="absolute inset-0 flex items-center justify-center">
            <div className="panel border-tc-red/40 bg-tc-red-dim">
              <span className="text-tc-red text-sm font-mono">
                ERR: {stopsQuery.error.message}
              </span>
            </div>
          </div>
        )}

        {/* Map */}
        {svgStops && (
          <>
            <span className="absolute bottom-3 left-3 z-10 text-xs font-mono text-tc-text-dim bg-tc-base/85 px-2 py-0.5 rounded border border-tc-border">
              {svgStops.filter((s) => s.stop_type === "required").length}{" "}
              required / {svgStops.length} stops
            </span>

            <svg
              width="100%"
              height="100%"
              className="block"
              role="img"
              aria-label="Station map"
            >
              <defs>
                <marker
                  id="arrowhead"
                  markerWidth="6"
                  markerHeight="4"
                  refX="5"
                  refY="2"
                  orient="auto"
                  markerUnits="strokeWidth"
                >
                  <polygon points="0 0, 6 2, 0 4" fill="#22c55e" />
                </marker>
              </defs>
              <g
                transform={`translate(${transform.x}, ${transform.y}) scale(${transform.scale})`}
              >
                {svgStops.map((stop) => (
                  <StopDot
                    key={stop.stop_id}
                    stop={stop}
                    scale={transform.scale}
                    isSelected={isSelected(stop.stop_id)}
                    isRequired={stop.stop_type === "required"}
                    isUnvisited={
                      visitedStopSet.size > 0 &&
                      !visitedStopSet.has(stop.stop_id)
                    }
                    isLeaf={leafSet.has(stop.stop_id)}
                    onClick={handleStopClick}
                  />
                ))}
                {/* Station name labels — greedy collision-avoidance placement */}
                {labelPlacements &&
                  svgStops.map((stop, i) => {
                    const lp = labelPlacements[i];
                    if (!lp.visible) return null;
                    return (
                      <text
                        key={`label-${stop.stop_id}`}
                        x={lp.dataX}
                        y={lp.dataY}
                        fontSize={LABEL_FONT_SIZE / transform.scale}
                        fontFamily='"SF Mono", "Cascadia Code", "JetBrains Mono", "Fira Code", ui-monospace, monospace'
                        fill="#5c6378"
                      >
                        {stop.stop_name}
                      </text>
                    );
                  })}
                {/* Partial solution: path arrows */}
                {pathArrows.map((a, i) => (
                  <line
                    key={`arrow-${i}`}
                    style={{ pointerEvents: "none" }}
                    x1={a.x1}
                    y1={a.y1}
                    x2={a.x2}
                    y2={a.y2}
                    stroke="#22c55e"
                    strokeWidth={2 / transform.scale}
                    markerEnd="url(#arrowhead)"
                    opacity={0.7}
                  />
                ))}
              </g>
            </svg>
          </>
        )}
      </div>

      {/* Overlay path panel — show when overlay is selected */}
      {partialQuery.data && (
        <OverlayPathPanel
          stopNames={stopNames}
          partialData={partialQuery.data}
        />
      )}

      {/* Station panel — outside the gesture-target div */}
      {stopsQuery.data && (
        <StationPanel
          name={name}
          stops={stopsQuery.data.filter((s) => s.stop_type === "required")}
          stopNames={stopNames}
          origin={origin}
          destination={destination}
          onOriginChange={setOrigin}
          onDestChange={setDestination}
          onOriginClear={() => setOrigin(null)}
          onDestClear={() => setDestination(null)}
          onSwap={() => {
            setOrigin(destination);
            setDestination(origin);
          }}
        />
      )}
    </div>
  );
}
