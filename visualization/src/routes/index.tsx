import { createFileRoute, Link } from "@tanstack/react-router";
import { trpc } from "../client/trpc";

export const Route = createFileRoute("/")({
  component: IndexPage,
});

function IndexPage() {
  const visualizationsQuery = trpc.listVisualizations.useQuery();

  return (
    <div
      style={{
        minHeight: "100vh",
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        justifyContent: "center",
        padding: "3rem 1.5rem",
      }}
    >
      <h1
        style={{
          fontSize: "1.5rem",
          fontWeight: 600,
          letterSpacing: "-0.02em",
          marginBottom: "2.5rem",
        }}
      >
        Visualizations
      </h1>

      {visualizationsQuery.isLoading && (
        <p style={{ color: "#888" }}>Loading...</p>
      )}
      {visualizationsQuery.error && (
        <p style={{ color: "#c00" }}>
          Error: {visualizationsQuery.error.message}
        </p>
      )}

      {visualizationsQuery.data && (
        <div
          style={{
            display: "flex",
            flexDirection: "column",
            gap: "0.5rem",
            width: "100%",
            maxWidth: "28rem",
          }}
        >
          {visualizationsQuery.data.map((vis) => (
            <Link
              key={vis.filename}
              to="/viz/$name"
              params={{ name: vis.name }}
              style={{
                display: "block",
                padding: "1rem 1.25rem",
                borderRadius: "0.5rem",
                border: "1px solid #e5e5e5",
                textDecoration: "none",
                color: "#1a1a1a",
                transition: "border-color 0.15s, box-shadow 0.15s",
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.borderColor = "#999";
                e.currentTarget.style.boxShadow = "0 1px 4px rgba(0,0,0,0.06)";
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.borderColor = "#e5e5e5";
                e.currentTarget.style.boxShadow = "none";
              }}
            >
              {vis.name}
            </Link>
          ))}
        </div>
      )}
    </div>
  );
}
