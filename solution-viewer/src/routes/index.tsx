import { createFileRoute } from "@tanstack/react-router";
import { trpc } from "../client/trpc";
import { usePageTitle } from "./__root";

export const Route = createFileRoute("/")({
  component: IndexPage,
});

function IndexPage() {
  usePageTitle("Solution Viewer");
  const helloWorldQuery = trpc.listHelloWorld.useQuery();

  return (
    <div className="min-h-screen px-6 py-12 flex justify-center">
      <div className="w-full max-w-2xl space-y-4">
        <header className="flex items-baseline justify-between border-b border-tc-border pb-2">
          <h1 className="font-mono text-sm tracking-widest uppercase text-tc-text-muted">
            hello_world
          </h1>
          {helloWorldQuery.data && (
            <span className="font-mono text-xs text-tc-text-dim">
              {helloWorldQuery.data.length} rows
            </span>
          )}
        </header>

        {helloWorldQuery.isPending && (
          <p className="text-tc-text-muted font-mono text-sm animate-pulse">Loading...</p>
        )}

        {helloWorldQuery.error && (
          <p className="text-tc-red text-sm font-mono">ERR: {helloWorldQuery.error.message}</p>
        )}

        {helloWorldQuery.data && (
          <table className="w-full font-mono text-sm border-collapse">
            <thead>
              <tr className="text-left text-tc-text-dim text-xs uppercase tracking-wider">
                <th className="py-1 pr-4 font-medium">id</th>
                <th className="py-1 pr-4 font-medium">message</th>
                <th className="py-1 font-medium">created_at</th>
              </tr>
            </thead>
            <tbody>
              {helloWorldQuery.data.map((row) => (
                <tr key={row.id} className="border-t border-tc-border">
                  <td className="py-1.5 pr-4 text-tc-cyan tabular-nums">{row.id}</td>
                  <td className="py-1.5 pr-4 text-tc-text">{row.message}</td>
                  <td className="py-1.5 text-tc-text-dim tabular-nums">
                    {new Date(row.created_at).toLocaleString()}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </div>
    </div>
  );
}
