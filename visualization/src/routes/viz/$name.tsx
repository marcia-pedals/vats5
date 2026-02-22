import { useMemo, useRef, useCallback, useState } from "react";
import { createFileRoute, Link } from "@tanstack/react-router";
import { useGesture } from "@use-gesture/react";
import { trpc } from "../../client/trpc";

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

function VizPage() {
  const { name } = Route.useParams();
  const filename = `${name}-viz.json`;

  const visualizationQuery = trpc.getVisualization.useQuery({ filename });
  const containerRef = useRef<HTMLDivElement>(null);
  const [transform, setTransform] = useState<Transform>(DEFAULT_TRANSFORM);
  const [containerSize, setContainerSize] = useState<{ w: number; h: number } | null>(null);

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
      cx: offsetX + ((stop.lon - minLon) * cosLat / dataW) * drawW,
      cy: offsetY + ((maxLat - stop.lat) / dataH) * drawH,
    }));
  }, [visualizationQuery.data, containerSize]);

  useGesture(
    {
      onDrag: ({ delta: [dx, dy] }) => {
        setTransform((t) => ({ ...t, x: t.x + dx, y: t.y + dy }));
      },
      onWheel: ({ event }) => {
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
          onClick={resetView}
          className="px-2.5 py-1 rounded text-xs font-mono text-tc-text-muted border border-tc-border bg-transparent hover:border-tc-cyan/50 hover:text-tc-cyan transition-colors cursor-pointer"
        >
          Reset view
        </button>
        <span className="text-xs font-mono text-tc-text-dim ml-2">
          {name}
        </span>
      </div>

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
          <span className="absolute bottom-3 right-3 z-10 text-xs font-mono text-tc-text-dim bg-tc-base/85 px-2 py-0.5 rounded border border-tc-border">
            {svgStops.length} stops
          </span>

          <svg
            width="100%"
            height="100%"
            className="block pointer-events-none"
          >
            <g transform={`translate(${transform.x}, ${transform.y}) scale(${transform.scale})`}>
              {svgStops.map((stop) => (
                <g key={stop.stop_id}>
                  <circle
                    cx={stop.cx}
                    cy={stop.cy}
                    r={DOT_R / transform.scale}
                    fill="#0094b3"
                    stroke="#006880"
                    strokeWidth={1 / transform.scale}
                    opacity={0.85}
                  />
                  <title>
                    {stop.stop_name} ({stop.stop_id})
                    {"\n"}Lat: {stop.lat.toFixed(5)}, Lon: {stop.lon.toFixed(5)}
                  </title>
                </g>
              ))}
            </g>
          </svg>
        </>
      )}
    </div>
  );
}
