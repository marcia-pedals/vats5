import { createFileRoute, Link } from "@tanstack/react-router";
import { trpc } from "../client/trpc";
import { useCheckoutTitle } from "./__root";

export const Route = createFileRoute("/")({
  component: IndexPage,
});

function IndexPage() {
  useCheckoutTitle("Visualizations");
  const visualizationsQuery = trpc.listVisualizations.useQuery();

  return (
    <div className="min-h-screen flex flex-col items-center justify-center px-6 py-12">
      <div className="w-full max-w-xs">
        {visualizationsQuery.isPending && (
          <p className="text-tc-text-muted font-mono text-sm animate-pulse">Loading...</p>
        )}

        {visualizationsQuery.error && (
          <p className="text-tc-red text-sm font-mono">ERR: {visualizationsQuery.error.message}</p>
        )}

        {visualizationsQuery.data && (
          <div className="font-mono text-sm space-y-0.5">
            {visualizationsQuery.data.map((vis) => (
              <Link
                key={vis.filename}
                to="/viz/$name"
                params={{ name: vis.name }}
                className="block px-2 py-1 -mx-2 rounded text-tc-text hover:text-tc-cyan hover:bg-tc-cyan/8 transition-colors no-underline"
              >
                {vis.name}
              </Link>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}
