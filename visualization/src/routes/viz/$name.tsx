import { createFileRoute, Link } from "@tanstack/react-router";
import { useGesture } from "@use-gesture/react";
import { useCallback, useMemo, useRef, useState } from "react";
import { trpc } from "../../client/trpc";
import type { VizPath, VizPathGroup } from "../../server/schemas";

export const Route = createFileRoute("/viz/$name")({
  component: VizPage,
});

const DOT_R = 5;

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

function PathTable({
  paths,
  originName,
  destName,
  onClose,
  panelRef,
}: {
  paths: VizPath[];
  originName: string;
  destName: string;
  onClose: () => void;
  panelRef: React.RefObject<HTMLDivElement | null>;
}) {
  return (
    <div
      ref={panelRef}
      className="absolute top-0 bottom-0 right-0 z-20 max-w-[480px] w-full flex flex-col panel-surface m-3 overflow-hidden"
    >
      {/* Header */}
      <div className="flex items-center justify-between gap-2 px-3 py-2 border-b border-tc-border shrink-0">
        <div className="flex items-center gap-1.5 min-w-0">
          <span className="text-xs font-mono text-tc-cyan font-semibold truncate">
            {originName}
          </span>
          <span className="text-xs font-mono text-tc-text-dim">&rarr;</span>
          <span className="text-xs font-mono text-tc-cyan font-semibold truncate">{destName}</span>
        </div>
        <div className="flex items-center gap-2 shrink-0">
          <span className="text-[10px] font-mono text-tc-text-dim">{paths.length} paths</span>
          <button
            type="button"
            onClick={onClose}
            className="px-1.5 py-0.5 rounded text-[10px] font-mono text-tc-text-muted border border-tc-border bg-transparent hover:border-tc-red/50 hover:text-tc-red transition-colors cursor-pointer"
          >
            close
          </button>
        </div>
      </div>

      {/* Table */}
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
    </div>
  );
}

function VizPage() {
  const { name } = Route.useParams();
  const filename = `${name}-viz.json`;

  const visualizationQuery = trpc.getVisualization.useQuery({ filename });
  const containerRef = useRef<HTMLDivElement>(null);
  const panelRef = useRef<HTMLDivElement>(null);
  const [transform, setTransform] = useState<Transform>(DEFAULT_TRANSFORM);
  const [containerSize, setContainerSize] = useState<{
    w: number;
    h: number;
  } | null>(null);
  const [selectedStops, setSelectedStops] = useState<string[]>([]);

  const containerCallbackRef = useCallback((node: HTMLDivElement | null) => {
    containerRef.current = node;
    if (node) {
      setContainerSize({ w: node.clientWidth, h: node.clientHeight });
    }
  }, []);

  const svgStops = useMemo(() => {
    if (!visualizationQuery.data?.stops.length || !containerSize) return null;

    const stops = visualizationQuery.data.stops;
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
  }, [visualizationQuery.data, containerSize]);

  // Build a lookup map for path groups: "origin:dest" -> VizPathGroup
  const pathLookup = useMemo(() => {
    if (!visualizationQuery.data?.path_groups) return null;
    const map = new Map<string, VizPathGroup>();
    for (const pg of visualizationQuery.data.path_groups) {
      map.set(`${pg.origin_stop_id}:${pg.destination_stop_id}`, pg);
    }
    return map;
  }, [visualizationQuery.data]);

  // Get paths between selected stops (in both directions)
  const selectedPaths = useMemo<{
    paths: VizPath[];
    originName: string;
    destName: string;
  } | null>(() => {
    if (selectedStops.length !== 2 || !pathLookup || !svgStops) return null;
    const [a, b] = selectedStops;
    const key = `${a}:${b}`;
    const pg = pathLookup.get(key);
    const stopA = svgStops.find((s) => s.stop_id === a);
    const stopB = svgStops.find((s) => s.stop_id === b);
    if (!pg || !stopA || !stopB) return null;
    return {
      paths: pg.paths,
      originName: stopA.stop_name,
      destName: stopB.stop_name,
    };
  }, [selectedStops, pathLookup, svgStops]);

  const handleStopClick = useCallback((stopId: string) => {
    setSelectedStops((prev) => {
      if (prev.length === 0) return [stopId];
      if (prev.length === 1) {
        if (prev[0] === stopId) return [];
        return [prev[0], stopId];
      }
      // Two already selected — start fresh with new selection
      return [stopId];
    });
  }, []);

  const isEventInPanel = useCallback((event: Event) => {
    const panel = panelRef.current;
    return panel != null && event.target instanceof Node && panel.contains(event.target);
  }, []);

  useGesture(
    {
      onDrag: ({ event, delta: [dx, dy] }) => {
        if (isEventInPanel(event)) return;
        setTransform((t) => ({ ...t, x: t.x + dx, y: t.y + dy }));
      },
      onWheel: ({ event }) => {
        if (isEventInPanel(event)) return;
        event.preventDefault();
        const we = event as WheelEvent;
        const container = containerRef.current;
        if (!container) return;
        const rect = container.getBoundingClientRect();
        const px = we.clientX - rect.left;
        const py = we.clientY - rect.top;

        setTransform((t) => {
          const factor = we.deltaY > 0 ? 1 / 1.1 : 1.1;
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
      target: containerRef,
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

  const isSelected = (stopId: string) => selectedStops.includes(stopId);

  return (
    <div
      ref={containerCallbackRef}
      className="relative w-screen h-screen overflow-hidden touch-none cursor-grab bg-tc-void"
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
        {selectedStops.length > 0 && (
          <button
            type="button"
            onClick={() => setSelectedStops([])}
            className="px-2.5 py-1 rounded text-xs font-mono text-tc-text-muted border border-tc-border bg-transparent hover:border-tc-red/50 hover:text-tc-red transition-colors cursor-pointer ml-1"
          >
            Clear selection
          </button>
        )}
      </div>

      {/* Selection hint */}
      {selectedStops.length === 1 && svgStops && (
        <div className="absolute top-3 right-3 z-10 panel-surface py-1.5 px-2.5">
          <span className="text-xs font-mono text-tc-text-muted">
            Select second stop to view paths from{" "}
            <span className="text-tc-cyan font-semibold">
              {svgStops.find((s) => s.stop_id === selectedStops[0])?.stop_name}
            </span>
          </span>
        </div>
      )}

      {/* Loading */}
      {visualizationQuery.isLoading && (
        <div className="absolute inset-0 flex items-center justify-center">
          <span className="text-tc-text-muted font-mono text-sm animate-pulse">
            Loading feed...
          </span>
        </div>
      )}

      {/* Error */}
      {visualizationQuery.error && (
        <div className="absolute inset-0 flex items-center justify-center">
          <div className="panel border-tc-red/40 bg-tc-red-dim">
            <span className="text-tc-red text-sm font-mono">
              ERR: {visualizationQuery.error.message}
            </span>
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
                    <title>
                      {stop.stop_name} ({stop.stop_id}){"\n"}Lat: {stop.lat.toFixed(5)}, Lon:{" "}
                      {stop.lon.toFixed(5)}
                    </title>
                  </g>
                );
              })}
            </g>
          </svg>
        </>
      )}

      {/* Path table */}
      {selectedPaths && (
        <PathTable
          paths={selectedPaths.paths}
          originName={selectedPaths.originName}
          destName={selectedPaths.destName}
          onClose={() => setSelectedStops([])}
          panelRef={panelRef}
        />
      )}

      {/* No paths found message */}
      {selectedStops.length === 2 && !selectedPaths && (
        <div className="absolute bottom-3 right-3 z-20 panel-surface py-1.5 px-2.5">
          <span className="text-xs font-mono text-tc-text-dim">
            No paths found between these stops
          </span>
        </div>
      )}
    </div>
  );
}
