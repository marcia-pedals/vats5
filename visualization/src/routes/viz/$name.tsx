import { useMemo, useRef, useCallback, useState, useLayoutEffect } from "react";
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

  useLayoutEffect(() => {
    if (containerRef.current) {
      setContainerSize({
        w: containerRef.current.clientWidth,
        h: containerRef.current.clientHeight,
      });
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

    // Use uniform scale to preserve geographic aspect ratio
    // cos(lat) corrects for longitude compression at higher latitudes
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
        // Mouse position relative to container
        const px = we.clientX - rect.left;
        const py = we.clientY - rect.top;

        setTransform((t) => {
          const factor = we.deltaY > 0 ? 1 / 1.1 : 1.1;
          const newScale = Math.min(Math.max(t.scale * factor, 0.1), 100);
          // Adjust translation so the point under the cursor stays fixed:
          // Before: screenPoint = point * oldScale + oldTranslate
          // After:  screenPoint = point * newScale + newTranslate
          // => newTranslate = screenPoint - (screenPoint - oldTranslate) * (newScale / oldScale)
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

  const btnStyle: React.CSSProperties = {
    background: "none",
    border: "1px solid #ddd",
    borderRadius: "0.25rem",
    padding: "0.3rem 0.6rem",
    fontSize: "0.75rem",
    color: "#555",
    cursor: "pointer",
  };

  return (
    <div
      ref={containerRef}
      style={{
        position: "relative",
        width: "100vw",
        height: "100vh",
        overflow: "hidden",
        touchAction: "none",
        cursor: "grab",
      }}
    >
      {/* Control menu */}
      <div
        style={{
          position: "absolute",
          top: "0.75rem",
          left: "0.75rem",
          zIndex: 10,
          display: "flex",
          gap: "0.4rem",
          alignItems: "center",
          background: "rgba(255,255,255,0.9)",
          padding: "0.35rem 0.5rem",
          borderRadius: "0.375rem",
          border: "1px solid #e5e5e5",
        }}
      >
        <Link
          to="/"
          style={{
            ...btnStyle,
            textDecoration: "none",
            display: "inline-flex",
            alignItems: "center",
          }}
        >
          &larr; back
        </Link>
        <button style={btnStyle} onClick={resetView}>
          Reset view
        </button>
      </div>

      {visualizationQuery.isLoading && (
        <div
          style={{
            position: "absolute",
            inset: 0,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            color: "#888",
          }}
        >
          Loading...
        </div>
      )}

      {visualizationQuery.error && (
        <div
          style={{
            position: "absolute",
            inset: 0,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            color: "#c00",
          }}
        >
          Error: {visualizationQuery.error.message}
        </div>
      )}

      {svgStops && (
        <>
          <span
            style={{
              position: "absolute",
              bottom: "0.75rem",
              right: "0.75rem",
              zIndex: 10,
              fontSize: "0.75rem",
              color: "#999",
              background: "rgba(255,255,255,0.85)",
              padding: "0.2rem 0.5rem",
              borderRadius: "0.25rem",
            }}
          >
            {svgStops.length} stops
          </span>

          <svg
            width="100%"
            height="100%"
            style={{ display: "block", pointerEvents: "none" }}
          >
            <g transform={`translate(${transform.x}, ${transform.y}) scale(${transform.scale})`}>
              {svgStops.map((stop) => (
                <g key={stop.stop_id}>
                  <circle
                    cx={stop.cx}
                    cy={stop.cy}
                    r={DOT_R / transform.scale}
                    fill="#2563eb"
                    stroke="#1e40af"
                    strokeWidth={1 / transform.scale}
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
