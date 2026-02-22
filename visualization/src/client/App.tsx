import { useState, useMemo } from "react";
import { trpc } from "./trpc";

interface Stop {
  stop_id: string;
  stop_name: string;
  lat: number;
  lon: number;
}

function App() {
  const [selectedFile, setSelectedFile] = useState<string | null>(null);

  const visualizationsQuery = trpc.listVisualizations.useQuery();
  const visualizationQuery = trpc.getVisualization.useQuery(
    { filename: selectedFile! },
    { enabled: !!selectedFile }
  );

  // Compute SVG coordinates with automatic scaling
  const svgData = useMemo(() => {
    if (!visualizationQuery.data?.stops.length) return null;

    const stops = visualizationQuery.data.stops;

    // Find bounds
    const lats = stops.map((s) => s.lat);
    const lons = stops.map((s) => s.lon);
    const minLat = Math.min(...lats);
    const maxLat = Math.max(...lats);
    const minLon = Math.min(...lons);
    const maxLon = Math.max(...lons);

    // SVG dimensions
    const width = 800;
    const height = 600;
    const padding = 40;

    // Scale functions (note: latitude is flipped for SVG coordinates)
    const latRange = maxLat - minLat || 1;
    const lonRange = maxLon - minLon || 1;

    const scaleX = (lon: number) =>
      padding + ((lon - minLon) / lonRange) * (width - 2 * padding);
    const scaleY = (lat: number) =>
      padding + ((maxLat - lat) / latRange) * (height - 2 * padding);

    return {
      width,
      height,
      stops: stops.map((stop) => ({
        ...stop,
        x: scaleX(stop.lon),
        y: scaleY(stop.lat),
      })),
    };
  }, [visualizationQuery.data]);

  return (
    <div style={{ padding: "2rem", fontFamily: "system-ui" }}>
      <h1>VATS5 Visualization Viewer</h1>

      <div style={{ marginTop: "2rem" }}>
        <h2>Select Visualization</h2>
        {visualizationsQuery.isLoading && <p>Loading visualizations...</p>}
        {visualizationsQuery.error && (
          <p style={{ color: "red" }}>Error: {visualizationsQuery.error.message}</p>
        )}
        {visualizationsQuery.data && (
          <select
            value={selectedFile || ""}
            onChange={(e) => setSelectedFile(e.target.value || null)}
            style={{ padding: "0.5rem", fontSize: "1rem", minWidth: "300px" }}
          >
            <option value="">-- Select a visualization --</option>
            {visualizationsQuery.data.map((vis) => (
              <option key={vis.filename} value={vis.filename}>
                {vis.name}
              </option>
            ))}
          </select>
        )}
      </div>

      {selectedFile && (
        <div style={{ marginTop: "2rem" }}>
          <h2>{selectedFile.replace("-vis.json", "")}</h2>
          {visualizationQuery.isLoading && <p>Loading visualization...</p>}
          {visualizationQuery.error && (
            <p style={{ color: "red" }}>Error: {visualizationQuery.error.message}</p>
          )}
          {svgData && (
            <div>
              <p style={{ color: "#666", marginBottom: "1rem" }}>
                Showing {svgData.stops.length} stops
              </p>
              <svg
                width={svgData.width}
                height={svgData.height}
                style={{ border: "1px solid #ccc", background: "#f9f9f9" }}
              >
                {svgData.stops.map((stop) => (
                  <g key={stop.stop_id}>
                    <circle
                      cx={stop.x}
                      cy={stop.y}
                      r={5}
                      fill="#2563eb"
                      stroke="#1e40af"
                      strokeWidth={1}
                    />
                    <title>
                      {stop.stop_name} ({stop.stop_id})
                      {"\n"}Lat: {stop.lat.toFixed(5)}, Lon: {stop.lon.toFixed(5)}
                    </title>
                  </g>
                ))}
              </svg>
            </div>
          )}
        </div>
      )}
    </div>
  );
}

export default App;
