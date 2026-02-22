import { skipToken } from "@tanstack/react-query";
import { createFileRoute, Link } from "@tanstack/react-router";
import { useGesture } from "@use-gesture/react";
import { ArrowUpDown, X } from "lucide-react";
import { useCallback, useMemo, useRef, useState } from "react";
import { trpc } from "../../client/trpc";
import type { Stop, VizPath } from "../../server/db";

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
  if (h > 0) return `${h}h ${m}m`;
  return `${m}m`;
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
      <span className="text-[10px] font-mono text-tc-text-dim w-8 shrink-0">{label}</span>
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
        disabled={!value}
        className="w-5 h-5 flex items-center justify-center rounded text-tc-text-dim border border-tc-border bg-transparent hover:border-tc-red/50 hover:text-tc-red transition-colors cursor-pointer disabled:opacity-0 disabled:cursor-default shrink-0"
        aria-label={`Clear ${label.toLowerCase()}`}
      >
        <X size={10} />
      </button>
    </div>
  );
}

function StationPanel({
  name,
  stops,
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
  origin: string | null;
  destination: string | null;
  onOriginChange: (stopId: string) => void;
  onDestChange: (stopId: string) => void;
  onOriginClear: () => void;
  onDestClear: () => void;
  onSwap: () => void;
}) {
  const pathsQuery = trpc.getPaths.useQuery(
    origin && destination ? { name, origin, destination } : skipToken
  );
  const paths: VizPath[] | null = pathsQuery.data ?? null;

  return (
    <div className="absolute top-0 bottom-0 right-0 z-20 w-[340px] flex flex-col panel-surface m-3 overflow-hidden">
      {/* Station selectors */}
      <div className="flex items-center gap-2 px-3 py-2.5 border-b border-tc-border shrink-0">
        <button
          type="button"
          onClick={onSwap}
          disabled={!origin && !destination}
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
                  <th className="text-left px-2 py-1.5 font-normal">Depart</th>
                  <th className="text-left px-2 py-1.5 font-normal">Arrive</th>
                  <th className="text-right px-2 py-1.5 font-normal">Duration</th>
                  <th className="text-center px-2 py-1.5 font-normal">Flex</th>
                </tr>
              </thead>
              <tbody>
                {paths.map((p) => (
                  <tr
                    key={`${p.depart_time}-${p.arrive_time}`}
                    className="border-b border-tc-border/50 hover:bg-tc-cyan-dim/40 transition-colors"
                  >
                    <td className="px-2 py-1.5 text-tc-text">{formatTime(p.depart_time)}</td>
                    <td className="px-2 py-1.5 text-tc-text">{formatTime(p.arrive_time)}</td>
                    <td className="px-2 py-1.5 text-right text-tc-text-muted">
                      {formatDuration(p.duration_seconds)}
                    </td>
                    <td className="px-2 py-1.5 text-center">
                      {p.is_flex ? (
                        <span className="text-tc-amber">flex</span>
                      ) : (
                        <span className="text-tc-text-dim">--</span>
                      )}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </>
      )}

      {/* No paths message */}
      {origin && destination && paths !== null && paths.length === 0 && (
        <div className="px-3 py-3 text-xs font-mono text-tc-text-dim">
          No paths found between these stops
        </div>
      )}

      {/* Empty state */}
      {(!origin || !destination) && (
        <div className="px-3 py-3 text-xs font-mono text-tc-text-dim">
          {!origin
            ? "Click a station on the map to set origin"
            : "Click a station on the map to set destination"}
        </div>
      )}
    </div>
  );
}

function VizPage() {
  const { name } = Route.useParams();

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

  const containerCallbackRef = useCallback((node: HTMLDivElement | null) => {
    containerRef.current = node;
    if (node) {
      setContainerSize({ w: node.clientWidth, h: node.clientHeight });
    }
  }, []);

  const svgStops = useMemo(() => {
    if (!stopsQuery.data?.length || !containerSize) return null;

    const stops = stopsQuery.data;
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
    const scalePerPixel = Math.min((w - 2 * pad) / dataW, (h - 2 * pad) / dataH);

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
    [origin]
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
    const order = sp.map((_, i) => i).sort((a, b) => nbrDistSq[b] - nbrDistSq[a]);

    const boxes: [number, number, number, number][] = []; // placed bboxes
    const out: { dataX: number; dataY: number; visible: boolean }[] = new Array(svgStops.length);

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
    }
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

  const selectedStops = useMemo(() => {
    const s = new Set<string>();
    if (origin) s.add(origin);
    if (destination) s.add(destination);
    return s;
  }, [origin, destination]);

  const isSelected = (stopId: string) => selectedStops.has(stopId);

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
              <span className="text-tc-red text-sm font-mono">ERR: {stopsQuery.error.message}</span>
            </div>
          </div>
        )}

        {/* Map */}
        {svgStops && (
          <>
            <span className="absolute bottom-3 left-3 z-10 text-xs font-mono text-tc-text-dim bg-tc-base/85 px-2 py-0.5 rounded border border-tc-border">
              {svgStops.length} stops
            </span>

            <svg width="100%" height="100%" className="block" role="img" aria-label="Station map">
              <g transform={`translate(${transform.x}, ${transform.y}) scale(${transform.scale})`}>
                {svgStops.map((stop) => {
                  const selected = isSelected(stop.stop_id);
                  return (
                    // biome-ignore lint/a11y/noStaticElementInteractions: SVG <g> cannot be a <button>
                    <g
                      key={stop.stop_id}
                      tabIndex={0}
                      onClick={(e) => {
                        e.stopPropagation();
                        handleStopClick(stop.stop_id);
                      }}
                      onKeyDown={(e) => {
                        if (e.key === "Enter" || e.key === " ") handleStopClick(stop.stop_id);
                      }}
                      style={{ cursor: "pointer", pointerEvents: "all" }}
                    >
                      {selected && (
                        <circle
                          cx={stop.cx}
                          cy={stop.cy}
                          r={(DOT_R + 4) / transform.scale}
                          fill="none"
                          stroke="#0094b3"
                          strokeWidth={2 / transform.scale}
                          opacity={0.6}
                        />
                      )}
                      <circle
                        cx={stop.cx}
                        cy={stop.cy}
                        r={DOT_R / transform.scale}
                        fill={selected ? "#00c4e8" : "#0094b3"}
                        stroke={selected ? "#0094b3" : "#006880"}
                        strokeWidth={1 / transform.scale}
                        opacity={0.85}
                      />
                    </g>
                  );
                })}
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
              </g>
            </svg>
          </>
        )}
      </div>

      {/* Station panel — outside the gesture-target div */}
      {stopsQuery.data && (
        <StationPanel
          name={name}
          stops={stopsQuery.data}
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
