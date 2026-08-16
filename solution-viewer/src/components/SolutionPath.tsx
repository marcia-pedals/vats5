import { useCallback, useMemo, useState } from "react";
import type { PathStep, SolutionPath, SolutionRoute, SolutionStop } from "../schemas";

/**
 * The solution path of one problem instance: a map of the tour and the legs it
 * is made of. Mirrors the overlay in `visualization/` — same projection, same
 * greedy label placement, same green arrows — over the stop, route and path
 * metadata that iterative_expansion reports in its solution JSON.
 */

const MAP_HEIGHT = 420;
const MAP_PAD = 34; // px of margin around the projected stops
const DOT_R = 4; // constant screen-px
const LABEL_FONT_SIZE = 9; // constant screen-px
const LABEL_CHAR_W = 5.5; // approx monospace char width at 9px
const LABEL_PAD = 2; // collision padding (screen px)
const LABEL_MIN_NEIGHBOR = 13; // suppress a label whose nearest dot is closer

// Arrow colors, as CSS variable values so SVG matches the rest of the theme.
// Each is set on the line and, via its own marker, on the arrowhead.
const PATH_COLOR = "var(--color-tc-green)";
const WALK_COLOR = "var(--color-tc-cyan)";
const HIGHLIGHT_COLOR = "var(--color-tc-magenta)";

/** 64620 -> "17:57". Seconds since service start, so hours can pass 24. */
export function formatTimeOfDay(seconds: number): string {
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  return `${String(hours).padStart(2, "0")}:${String(minutes).padStart(2, "0")}`;
}

/** 21300 -> "5h55m", 2040 -> "34m". */
export function formatSpan(seconds: number): string {
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.round((seconds % 3600) / 60);
  return hours === 0 ? `${minutes}m` : `${hours}h${String(minutes).padStart(2, "0")}m`;
}

type Transform = { x: number; y: number; scale: number };

const DEFAULT_TRANSFORM: Transform = { x: 0, y: 0, scale: 1 };

type PlacedStop = SolutionStop & { cx: number; cy: number };

/**
 * Fit every stop into `width` x `height`, correcting longitude by the cosine
 * of the mid latitude so the shape is not stretched east-west.
 */
function projectStops(stops: SolutionStop[], width: number, height: number): PlacedStop[] {
  const lats = stops.map((stop) => stop.lat);
  const lons = stops.map((stop) => stop.lon);
  const minLat = Math.min(...lats);
  const maxLat = Math.max(...lats);
  const minLon = Math.min(...lons);
  const maxLon = Math.max(...lons);

  const cosLat = Math.cos((((minLat + maxLat) / 2) * Math.PI) / 180);
  const dataW = (maxLon - minLon || 1) * cosLat;
  const dataH = maxLat - minLat || 1;

  const scalePerPixel = Math.min((width - 2 * MAP_PAD) / dataW, (height - 2 * MAP_PAD) / dataH);
  const drawW = dataW * scalePerPixel;
  const drawH = dataH * scalePerPixel;
  const offsetX = MAP_PAD + (width - 2 * MAP_PAD - drawW) / 2;
  const offsetY = MAP_PAD + (height - 2 * MAP_PAD - drawH) / 2;

  return stops.map((stop) => ({
    ...stop,
    cx: offsetX + (((stop.lon - minLon) * cosLat) / dataW) * drawW,
    cy: offsetY + ((maxLat - stop.lat) / dataH) * drawH,
  }));
}

type LabelPlacement = { dataX: number; dataY: number; visible: boolean };

/**
 * Greedy label placement for the required stops: most isolated first, right of
 * the dot if that box is free and left otherwise, suppressed in dense
 * clusters. Placement is in screen space, so it is redone as the map zooms.
 */
function placeLabels(placed: PlacedStop[], scale: number): LabelPlacement[] {
  const screen = placed.map((stop) => ({ x: stop.cx * scale, y: stop.cy * scale }));
  const out: LabelPlacement[] = new Array(placed.length);
  const hidden: LabelPlacement = { dataX: 0, dataY: 0, visible: false };

  // Nearest-neighbor distance (squared) for each stop, over all dots.
  const nearestSq = screen.map((point, i) => {
    let nearest = Infinity;
    for (let j = 0; j < screen.length; j++) {
      if (i === j) continue;
      const dx = point.x - screen[j].x;
      const dy = point.y - screen[j].y;
      nearest = Math.min(nearest, dx * dx + dy * dy);
    }
    return nearest;
  });

  const order = placed
    .map((_, i) => i)
    .filter((i) => placed[i].required)
    .sort((a, b) => nearestSq[b] - nearestSq[a]);
  for (let i = 0; i < placed.length; i++) out[i] = hidden;

  const boxes: [number, number, number, number][] = [];
  for (const i of order) {
    if (nearestSq[i] < LABEL_MIN_NEIGHBOR * LABEL_MIN_NEIGHBOR) continue;

    const { x, y } = screen[i];
    const labelW = LABEL_CHAR_W * placed[i].name.length;
    const gap = 4;
    const candidates: [number, number, number, number][] = [
      [x + DOT_R + gap, y - LABEL_FONT_SIZE / 2, x + DOT_R + gap + labelW, y + LABEL_FONT_SIZE / 2],
      [x - DOT_R - gap - labelW, y - LABEL_FONT_SIZE / 2, x - DOT_R - gap, y + LABEL_FONT_SIZE / 2],
    ];

    for (const [bx1, by1, bx2, by2] of candidates) {
      const px1 = bx1 - LABEL_PAD;
      const py1 = by1 - LABEL_PAD;
      const px2 = bx2 + LABEL_PAD;
      const py2 = by2 + LABEL_PAD;

      let hit = boxes.some(
        ([ox1, oy1, ox2, oy2]) => px1 < ox2 && px2 > ox1 && py1 < oy2 && py2 > oy1
      );
      if (!hit) {
        // Also keep labels off other stops' dots.
        for (let j = 0; j < screen.length; j++) {
          if (j === i) continue;
          const nx = Math.max(px1, Math.min(screen[j].x, px2));
          const ny = Math.max(py1, Math.min(screen[j].y, py2));
          if ((screen[j].x - nx) ** 2 + (screen[j].y - ny) ** 2 < (DOT_R + 1) ** 2) {
            hit = true;
            break;
          }
        }
      }
      if (!hit) {
        out[i] = {
          dataX: bx1 / scale,
          dataY: (by1 + 0.75 * LABEL_FONT_SIZE) / scale,
          visible: true,
        };
        boxes.push([px1, py1, px2, py2]);
        break;
      }
    }
  }
  return out;
}

/** Track a container's width, so the projection can fit it. */
function useContainerWidth(): [(node: HTMLDivElement | null) => void, number] {
  const [width, setWidth] = useState(0);
  const ref = useCallback((node: HTMLDivElement | null) => {
    if (!node) return;
    setWidth(node.clientWidth);
    const observer = new ResizeObserver((entries) => setWidth(entries[0].contentRect.width));
    observer.observe(node);
    return () => observer.disconnect();
  }, []);
  return [ref, width];
}

/**
 * An arrowhead in one fixed color. A head cannot read the color off the line
 * it sits on -- `context-stroke` would, but Safari does not support it and
 * paints the head black -- so there is one marker per color a line can be.
 */
function ArrowMarker({ id, color }: { id: string; color: string }) {
  return (
    <marker
      id={id}
      markerWidth="6"
      markerHeight="4"
      refX="5"
      refY="2"
      orient="auto"
      markerUnits="strokeWidth"
    >
      <polygon points="0 0, 6 2, 0 4" fill={color} />
    </marker>
  );
}

function PathMap({
  stops,
  path,
  highlight,
}: {
  stops: SolutionStop[];
  path: SolutionPath;
  /** Half-open range of `original_steps` to draw as highlighted. */
  highlight: [number, number] | null;
}) {
  const [containerRef, width] = useContainerWidth();
  const [transform, setTransform] = useState<Transform>(DEFAULT_TRANSFORM);
  const [dragging, setDragging] = useState(false);

  const placed = useMemo(
    () => (width > 0 ? projectStops(stops, width, MAP_HEIGHT) : null),
    [stops, width]
  );
  const labels = useMemo(
    () => (placed ? placeLabels(placed, transform.scale) : null),
    [placed, transform.scale]
  );
  const coords = useMemo(() => {
    const map = new Map<string, { cx: number; cy: number }>();
    for (const stop of placed ?? []) map.set(stop.id, { cx: stop.cx, cy: stop.cy });
    return map;
  }, [placed]);

  // Every stop the path actually stops at, so the tour reads apart from the
  // stops it merely could have used.
  const onPath = useMemo(() => {
    const ids = new Set<string>();
    for (const step of path.original_steps) {
      ids.add(step.origin_stop_id);
      ids.add(step.destination_stop_id);
    }
    return ids;
  }, [path]);

  const arrows = useMemo(() => {
    if (!coords.size) return [];
    return path.original_steps.flatMap((step, index) => {
      const from = coords.get(step.origin_stop_id);
      const to = coords.get(step.destination_stop_id);
      if (!from || !to) return [];
      const highlighted = highlight !== null && index >= highlight[0] && index < highlight[1];
      return [
        {
          key: `${index}-${step.origin_stop_id}-${step.destination_stop_id}`,
          x1: from.cx,
          y1: from.cy,
          x2: to.cx,
          y2: to.cy,
          walk: step.is_flex !== 0,
          highlighted,
        },
      ];
    });
  }, [path, coords, highlight]);

  // Highlighted arrows last so they are not hidden under the rest of the tour.
  const orderedArrows = useMemo(
    () => [...arrows].sort((a, b) => Number(a.highlighted) - Number(b.highlighted)),
    [arrows]
  );

  /**
   * A wheel over the map zooms it and nothing else: the page must not scroll
   * out from under the cursor. React's own onWheel is registered passive and so
   * cannot preventDefault, so the listener is attached by hand -- from a ref
   * callback, since the svg mounts only once the container has been measured.
   */
  const svgRef = useCallback((svg: SVGSVGElement | null) => {
    if (!svg) return;

    const onWheel = (event: WheelEvent) => {
      event.preventDefault();
      const rect = svg.getBoundingClientRect();
      const px = event.clientX - rect.left;
      const py = event.clientY - rect.top;
      setTransform((current) => {
        // Dampen the mouse wheel's large deltas; leave trackpads alone.
        const delta = Math.abs(event.deltaY) > 50 ? event.deltaY * 0.3 : event.deltaY;
        const scale = Math.min(Math.max(current.scale * 1.005 ** -delta, 0.5), 40);
        return {
          x: px - (px - current.x) * (scale / current.scale),
          y: py - (py - current.y) * (scale / current.scale),
          scale,
        };
      });
    };

    svg.addEventListener("wheel", onWheel, { passive: false });
    return () => svg.removeEventListener("wheel", onWheel);
  }, []);

  // Panned or zoomed away from the fitted view, and so with something to reset.
  const moved =
    transform.x !== DEFAULT_TRANSFORM.x ||
    transform.y !== DEFAULT_TRANSFORM.y ||
    transform.scale !== DEFAULT_TRANSFORM.scale;

  // Where the tour begins and ends, which the arrows alone do not say.
  const endpoints = useMemo(() => {
    const first = path.steps[0];
    const last = path.steps[path.steps.length - 1];
    if (!first || !last) return [];
    return [
      { role: "start", color: PATH_COLOR, at: coords.get(first.origin_stop_id) },
      { role: "end", color: "var(--color-tc-amber)", at: coords.get(last.destination_stop_id) },
    ].flatMap((endpoint) => (endpoint.at ? [{ ...endpoint, at: endpoint.at }] : []));
  }, [path, coords]);

  return (
    <div
      ref={containerRef}
      className="relative overflow-hidden rounded-panel border border-tc-border bg-tc-base"
      style={{ height: MAP_HEIGHT }}
    >
      {placed && (
        <svg
          ref={svgRef}
          width="100%"
          height="100%"
          // Not selectable: dragging the map is a pan, not a text selection of
          // the stop labels it drags past.
          className={`block touch-none select-none ${dragging ? "cursor-grabbing" : "cursor-grab"}`}
          role="img"
          aria-label="Solution path map"
          onPointerDown={(event) => {
            event.currentTarget.setPointerCapture(event.pointerId);
            setDragging(true);
          }}
          onPointerMove={(event) => {
            if (!dragging) return;
            setTransform((current) => ({
              ...current,
              x: current.x + event.movementX,
              y: current.y + event.movementY,
            }));
          }}
          onPointerUp={() => setDragging(false)}
          onPointerCancel={() => setDragging(false)}
        >
          <defs>
            <ArrowMarker id="path-arrow" color={PATH_COLOR} />
            <ArrowMarker id="path-arrow-walk" color={WALK_COLOR} />
            <ArrowMarker id="path-arrow-highlight" color={HIGHLIGHT_COLOR} />
          </defs>
          <g transform={`translate(${transform.x}, ${transform.y}) scale(${transform.scale})`}>
            {placed.map((stop) => (
              <circle
                key={stop.id}
                cx={stop.cx}
                cy={stop.cy}
                r={(stop.required ? DOT_R : DOT_R - 1.5) / transform.scale}
                fill={stop.required ? "var(--color-tc-cyan)" : "var(--color-tc-text-dim)"}
                stroke={stop.required ? "var(--color-tc-blue)" : "var(--color-tc-text-muted)"}
                strokeWidth={1 / transform.scale}
                opacity={stop.required || onPath.has(stop.id) ? 0.85 : 0.4}
              >
                <title>{stop.name}</title>
              </circle>
            ))}

            {endpoints.map((endpoint) => (
              <circle
                key={endpoint.role}
                cx={endpoint.at.cx}
                cy={endpoint.at.cy}
                r={(DOT_R + 4) / transform.scale}
                fill="none"
                stroke={endpoint.color}
                strokeWidth={2 / transform.scale}
              >
                <title>{endpoint.role}</title>
              </circle>
            ))}

            {orderedArrows.map((arrow) => {
              const color = arrow.highlighted
                ? HIGHLIGHT_COLOR
                : arrow.walk
                  ? WALK_COLOR
                  : PATH_COLOR;
              const marker = arrow.highlighted
                ? "path-arrow-highlight"
                : arrow.walk
                  ? "path-arrow-walk"
                  : "path-arrow";
              return (
                <line
                  // Keyed by marker too, so a leg going in or out of the
                  // highlight remounts rather than being patched in place:
                  // Safari before 26.4 does not repaint a marker when
                  // `marker-end` changes on a live element, and would keep the
                  // magenta arrowhead after the line had gone back to green.
                  key={`${arrow.key}-${marker}`}
                  x1={arrow.x1}
                  y1={arrow.y1}
                  x2={arrow.x2}
                  y2={arrow.y2}
                  stroke={color}
                  // Constant: the arrowheads are markerUnits="strokeWidth", so
                  // a thicker hovered line would resize its head too.
                  strokeWidth={2 / transform.scale}
                  markerEnd={`url(#${marker})`}
                  strokeDasharray={
                    arrow.walk ? `${4 / transform.scale} ${3 / transform.scale}` : undefined
                  }
                  opacity={arrow.highlighted ? 0.95 : 0.7}
                  style={{ pointerEvents: "none" }}
                />
              );
            })}

            {labels?.map((label, index) =>
              label.visible ? (
                <text
                  key={placed[index].id}
                  x={label.dataX}
                  y={label.dataY}
                  fontSize={LABEL_FONT_SIZE / transform.scale}
                  fontFamily="var(--font-mono)"
                  fill="var(--color-tc-text-muted)"
                  style={{ pointerEvents: "none" }}
                >
                  {placed[index].name}
                </text>
              ) : null
            )}
          </g>
        </svg>
      )}

      {/* Only once there is something to reset to; on an untouched map the
          button is a control for a state you are already in. */}
      {moved && (
        <button
          type="button"
          className="absolute right-2 top-2 rounded-panel border border-tc-border bg-tc-raised px-2 py-0.5 font-mono text-[10px] text-tc-text-muted hover:border-tc-border-bright hover:text-tc-text cursor-pointer"
          onClick={() => setTransform(DEFAULT_TRANSFORM)}
        >
          reset view
        </button>
      )}
    </div>
  );
}

/**
 * Which `original_steps` each collapsed leg is made of. The collapse groups
 * consecutive steps on one trip, so walking the two lists together in order
 * recovers the ranges: a leg ends at the first step that reaches its
 * destination.
 */
function legStepRanges(path: SolutionPath): [number, number][] {
  const ranges: [number, number][] = [];
  let index = 0;
  for (const leg of path.steps) {
    const start = index;
    while (
      index < path.original_steps.length &&
      path.original_steps[index].destination_stop_id !== leg.destination_stop_id
    ) {
      index++;
    }
    index = Math.min(index + 1, path.original_steps.length);
    ranges.push([start, index]);
  }
  return ranges;
}

function LegRows({
  leg,
  stopNames,
  route,
  highlighted,
  onHover,
}: {
  leg: PathStep;
  stopNames: Map<string, string>;
  route?: SolutionRoute;
  highlighted: boolean;
  onHover: (hovering: boolean) => void;
}) {
  const rowClass = highlighted ? "bg-tc-magenta/10" : "";
  return (
    <tbody
      className={rowClass}
      onMouseEnter={() => onHover(true)}
      onMouseLeave={() => onHover(false)}
    >
      <tr>
        <td className="w-14 py-0.5 pl-2 align-top text-tc-text-dim tabular-nums">
          {formatTimeOfDay(leg.depart_time)}
        </td>
        <td className="py-0.5 pr-2">
          {leg.is_flex !== 0 ? (
            <span className="text-tc-cyan">Walk</span>
          ) : route ? (
            <span className="inline-flex items-center gap-1.5">
              {route.color && (
                <span
                  className="inline-block h-2 w-2 shrink-0 rounded-full"
                  style={{ backgroundColor: `#${route.color}` }}
                />
              )}
              <span className="text-tc-cyan">{route.name}</span>
            </span>
          ) : (
            <span className="text-tc-text-dim">{leg.route_direction_id ?? "unknown route"}</span>
          )}
          <span className="ml-2 text-tc-text-dim">
            {formatSpan(leg.arrive_time - leg.depart_time)}
          </span>
        </td>
      </tr>
      <tr className="border-b border-tc-border">
        <td className="w-14 py-0.5 pl-2 align-top text-tc-text-dim tabular-nums">
          {formatTimeOfDay(leg.arrive_time)}
        </td>
        <td className="py-0.5 pr-2 text-tc-text">
          {stopNames.get(leg.destination_stop_id) ?? leg.destination_stop_id}
        </td>
      </tr>
    </tbody>
  );
}

/** The map and the leg list for one solution path. */
export function SolutionPathView({
  stops,
  routes,
  path,
}: {
  stops: SolutionStop[];
  routes: SolutionRoute[];
  path: SolutionPath;
}) {
  const [hoveredLeg, setHoveredLeg] = useState<number | null>(null);

  const stopNames = useMemo(() => {
    const names = new Map<string, string>();
    for (const stop of stops) names.set(stop.id, stop.name);
    return names;
  }, [stops]);
  const routesById = useMemo(() => {
    const byId = new Map<string, SolutionRoute>();
    for (const route of routes) byId.set(route.id, route);
    return byId;
  }, [routes]);
  const ranges = useMemo(() => legStepRanges(path), [path]);

  if (path.steps.length === 0) {
    return <p className="font-mono text-xs text-tc-text-muted">The solution path is empty.</p>;
  }

  const first = path.steps[0];

  return (
    // Legs beside the map rather than under it: hovering a leg highlights it on
    // the map, and side by side both are on screen at once.
    <div className="flex items-start gap-3">
      <div
        className="w-64 shrink-0 overflow-y-auto rounded-panel border border-tc-border"
        style={{ maxHeight: MAP_HEIGHT }}
      >
        <table className="w-full border-collapse font-mono text-[11px]">
          <tbody>
            <tr className="border-b border-tc-border bg-tc-surface">
              <td className="w-14 py-1 pl-2 text-tc-text-dim">from</td>
              <td className="py-1 pr-2 text-tc-text">
                {stopNames.get(first.origin_stop_id) ?? first.origin_stop_id}
              </td>
            </tr>
          </tbody>
          {path.steps.map((leg, index) => (
            <LegRows
              // Legs are ordered and can repeat a stop pair, so the index is
              // the only stable identity here.
              // biome-ignore lint/suspicious/noArrayIndexKey: see above
              key={index}
              leg={leg}
              stopNames={stopNames}
              route={leg.route_direction_id ? routesById.get(leg.route_direction_id) : undefined}
              highlighted={hoveredLeg === index}
              onHover={(hovering) => setHoveredLeg(hovering ? index : null)}
            />
          ))}
        </table>
      </div>

      <div className="min-w-0 flex-1">
        <PathMap
          stops={stops}
          path={path}
          // `?? null` because a path swapped under a held hover can leave the
          // index past the end of the new path's legs.
          highlight={hoveredLeg === null ? null : (ranges[hoveredLeg] ?? null)}
        />
      </div>
    </div>
  );
}

/**
 * The gist of a path in one line: where it starts and ends, and how much of it
 * there is. Kept apart from the view above so the details card can sit it on
 * the same row as its own title rather than spending a row on it.
 */
export function SolutionPathSummary({
  stops,
  path,
}: {
  stops: SolutionStop[];
  path: SolutionPath;
}) {
  const stopNames = useMemo(() => {
    const names = new Map<string, string>();
    for (const stop of stops) names.set(stop.id, stop.name);
    return names;
  }, [stops]);

  const first = path.steps[0];
  const last = path.steps[path.steps.length - 1];
  if (!first || !last) {
    return null;
  }

  return (
    <span className="flex min-w-0 items-baseline gap-3 font-mono text-xs">
      {/* Matches the green and amber rings the map draws on these two. */}
      <span className="min-w-0 truncate text-tc-text-muted">
        <span className="text-tc-green">●</span> {formatTimeOfDay(first.depart_time)}{" "}
        {stopNames.get(first.origin_stop_id) ?? first.origin_stop_id} →{" "}
        <span className="text-tc-amber">●</span> {formatTimeOfDay(last.arrive_time)}{" "}
        {stopNames.get(last.destination_stop_id) ?? last.destination_stop_id}
      </span>
      <span className="whitespace-nowrap text-tc-text-dim">
        {path.steps.length} leg{path.steps.length === 1 ? "" : "s"} · {formatSpan(path.duration)}{" "}
        total
      </span>
    </span>
  );
}
