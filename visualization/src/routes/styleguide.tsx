import { createFileRoute, Link } from "@tanstack/react-router";

export const Route = createFileRoute("/styleguide")({
  component: StyleguidePage,
});

/* ── tiny helpers ── */

function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <section className="mb-12">
      <h2 className="text-xs font-mono uppercase tracking-widest text-tc-text-dim mb-4 border-b border-tc-border pb-2">
        {title}
      </h2>
      {children}
    </section>
  );
}

function Swatch({ name, className, hex }: { name: string; className: string; hex: string }) {
  return (
    <div className="flex items-center gap-3">
      <div className={`w-10 h-10 rounded border border-tc-border ${className}`} />
      <div>
        <div className="text-sm text-tc-text">{name}</div>
        <div className="text-xs font-mono text-tc-text-dim">{hex}</div>
      </div>
    </div>
  );
}

/* ── page ── */

function StyleguidePage() {
  return (
    <div className="min-h-screen bg-tc-void">
      {/* Header */}
      <header className="border-b border-tc-border bg-tc-base">
        <div className="max-w-5xl mx-auto px-6 py-6 flex items-center justify-between">
          <div>
            <h1 className="text-xl font-semibold tracking-tight text-tc-text">
              Control Center Design System
            </h1>
            <p className="text-sm text-tc-text-muted font-mono mt-0.5">
              Transit Control Center &mdash; Visual Language
            </p>
          </div>
          <Link
            to="/"
            className="text-xs font-mono text-tc-text-dim hover:text-tc-cyan transition-colors no-underline border border-tc-border rounded px-3 py-1.5 hover:border-tc-cyan/50"
          >
            &larr; Home
          </Link>
        </div>
      </header>

      <main className="max-w-5xl mx-auto px-6 py-10">

        {/* ─────────────── COLORS ─────────────── */}
        <Section title="Surfaces">
          <div className="grid grid-cols-2 sm:grid-cols-3 md:grid-cols-6 gap-3">
            <Swatch name="Void" className="bg-tc-void" hex="#f5f6f9" />
            <Swatch name="Base" className="bg-tc-base" hex="#edeef2" />
            <Swatch name="Raised" className="bg-tc-raised" hex="#ffffff" />
            <Swatch name="Surface" className="bg-tc-surface" hex="#f0f1f4" />
            <Swatch name="Overlay" className="bg-tc-overlay" hex="#e8e9ee" />
            <Swatch name="Subtle" className="bg-tc-subtle" hex="#dfe1e8" />
          </div>
        </Section>

        <Section title="Status Colors">
          <div className="grid grid-cols-2 sm:grid-cols-3 md:grid-cols-5 gap-4">
            <Swatch name="Green" className="bg-tc-green" hex="#0db340" />
            <Swatch name="Amber" className="bg-tc-amber" hex="#d69e00" />
            <Swatch name="Red" className="bg-tc-red" hex="#dc2a38" />
            <Swatch name="Cyan" className="bg-tc-cyan" hex="#0094b3" />
            <Swatch name="Blue" className="bg-tc-blue" hex="#2d6ae0" />
          </div>
        </Section>

        <Section title="Route Colors">
          <div className="grid grid-cols-2 sm:grid-cols-5 gap-4">
            <Swatch name="Yellow Line" className="bg-route-yellow" hex="#ffd000" />
            <Swatch name="Orange Line" className="bg-route-orange" hex="#ff8800" />
            <Swatch name="Red Line" className="bg-route-red" hex="#e22030" />
            <Swatch name="Green Line" className="bg-route-green" hex="#00b050" />
            <Swatch name="Blue Line" className="bg-route-blue" hex="#0080e0" />
          </div>
        </Section>

        {/* ─────────────── TYPOGRAPHY ─────────────── */}
        <Section title="Typography">
          <div className="space-y-6">
            <div className="panel">
              <p className="text-xs font-mono text-tc-text-dim mb-3">Display / Headings — font-sans</p>
              <h1 className="text-3xl font-semibold tracking-tight text-tc-text mb-2">System Overview</h1>
              <h2 className="text-xl font-semibold tracking-tight text-tc-text mb-2">Route Status</h2>
              <h3 className="text-lg font-medium text-tc-text mb-2">Station Details</h3>
              <h4 className="text-base font-medium text-tc-text-muted">Subsystem</h4>
            </div>
            <div className="panel">
              <p className="text-xs font-mono text-tc-text-dim mb-3">Monospace / Data — font-mono</p>
              <div className="space-y-2 font-mono">
                <p className="text-sm text-tc-green text-glow-green">SYS.OK &mdash; ALL LINES OPERATIONAL</p>
                <p className="text-sm text-tc-amber text-glow-amber">WARN &mdash; DELAY CENTRAL STN &rarr; UNION 4m</p>
                <p className="text-sm text-tc-red">ERR &mdash; SIGNAL FAULT AT WESTGATE</p>
                <p className="text-sm text-tc-cyan">INFO &mdash; NEXT TRAIN PLAZA 2m 14s</p>
                <p className="text-sm text-tc-text-muted">-- idle --</p>
              </div>
            </div>
            <div className="panel">
              <p className="text-xs font-mono text-tc-text-dim mb-3">Body Text — font-sans</p>
              <p className="text-sm text-tc-text leading-relaxed max-w-prose">
                The Transit Control Center monitors all vehicle movements across the
                network in real time. Controllers track position, speed, and
                dwell time for every revenue vehicle, coordinating signal priority
                and managing disruptions from a centralized operations room.
              </p>
            </div>
          </div>
        </Section>

        {/* ─────────────── PANELS ─────────────── */}
        <Section title="Panels">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            <div className="panel">
              <p className="text-xs font-mono text-tc-text-dim mb-2">panel (raised)</p>
              <p className="text-sm text-tc-text">Default raised panel for grouping content.</p>
            </div>
            <div className="panel-surface">
              <p className="text-xs font-mono text-tc-text-dim mb-2">panel-surface</p>
              <p className="text-sm text-tc-text">Surface-level panel for inline controls.</p>
            </div>
            <div className="panel scanline">
              <p className="text-xs font-mono text-tc-text-dim mb-2">panel + scanline</p>
              <p className="text-sm text-tc-text">Scanline overlay for a control-room feel.</p>
            </div>
            <div className="panel border-tc-cyan/30 glow-cyan">
              <p className="text-xs font-mono text-tc-text-dim mb-2">panel + glow-cyan</p>
              <p className="text-sm text-tc-text">Highlighted / active panel with glow effect.</p>
            </div>
          </div>
        </Section>

        {/* ─────────────── STATUS INDICATORS ─────────────── */}
        <Section title="Status Indicators">
          <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-4 gap-4">
            <div className="panel flex items-center gap-3">
              <span className="status-dot bg-tc-green animate-pulse" />
              <div>
                <div className="text-sm text-tc-text">Operational</div>
                <div className="text-xs font-mono text-tc-green">ALL CLEAR</div>
              </div>
            </div>
            <div className="panel flex items-center gap-3">
              <span className="status-dot bg-tc-amber" />
              <div>
                <div className="text-sm text-tc-text">Delayed</div>
                <div className="text-xs font-mono text-tc-amber">+4m DELAY</div>
              </div>
            </div>
            <div className="panel flex items-center gap-3">
              <span className="status-dot bg-tc-red animate-pulse" />
              <div>
                <div className="text-sm text-tc-text">Alert</div>
                <div className="text-xs font-mono text-tc-red">FAULT DET</div>
              </div>
            </div>
            <div className="panel flex items-center gap-3">
              <span className="status-dot bg-tc-text-dim" />
              <div>
                <div className="text-sm text-tc-text">Offline</div>
                <div className="text-xs font-mono text-tc-text-dim">NO SIGNAL</div>
              </div>
            </div>
          </div>
        </Section>

        {/* ─────────────── BUTTONS ─────────────── */}
        <Section title="Buttons">
          <div className="flex flex-wrap gap-3">
            <button className="px-4 py-2 rounded text-sm font-mono bg-tc-cyan/15 text-tc-cyan border border-tc-cyan/30 hover:bg-tc-cyan/25 hover:glow-cyan transition-all cursor-pointer">
              Primary
            </button>
            <button className="px-4 py-2 rounded text-sm font-mono bg-tc-green/15 text-tc-green border border-tc-green/30 hover:bg-tc-green/25 hover:glow-green transition-all cursor-pointer">
              Confirm
            </button>
            <button className="px-4 py-2 rounded text-sm font-mono bg-tc-amber/15 text-tc-amber border border-tc-amber/30 hover:bg-tc-amber/25 hover:glow-amber transition-all cursor-pointer">
              Caution
            </button>
            <button className="px-4 py-2 rounded text-sm font-mono bg-tc-red/15 text-tc-red border border-tc-red/30 hover:bg-tc-red/25 hover:glow-red transition-all cursor-pointer">
              Alert
            </button>
            <button className="px-4 py-2 rounded text-sm font-mono text-tc-text-muted border border-tc-border hover:border-tc-border-bright hover:text-tc-text transition-all cursor-pointer">
              Neutral
            </button>
            <button className="px-4 py-2 rounded text-sm font-mono text-tc-text-dim border border-transparent cursor-not-allowed opacity-50">
              Disabled
            </button>
          </div>
        </Section>

        {/* ─────────────── DATA TABLE ─────────────── */}
        <Section title="Data Table">
          <div className="panel overflow-x-auto">
            <table className="w-full text-sm font-mono">
              <thead>
                <tr className="border-b border-tc-border text-left">
                  <th className="py-2 pr-4 text-tc-text-dim font-normal text-xs uppercase tracking-wider">Station</th>
                  <th className="py-2 pr-4 text-tc-text-dim font-normal text-xs uppercase tracking-wider">Line</th>
                  <th className="py-2 pr-4 text-tc-text-dim font-normal text-xs uppercase tracking-wider">ETA</th>
                  <th className="py-2 text-tc-text-dim font-normal text-xs uppercase tracking-wider">Status</th>
                </tr>
              </thead>
              <tbody className="divide-y divide-tc-border/50">
                <tr>
                  <td className="py-2 pr-4 text-tc-text">Central Station</td>
                  <td className="py-2 pr-4"><span className="text-route-yellow">Yellow</span></td>
                  <td className="py-2 pr-4 text-tc-cyan">1m 42s</td>
                  <td className="py-2"><span className="text-tc-green">ON TIME</span></td>
                </tr>
                <tr>
                  <td className="py-2 pr-4 text-tc-text">Union Square</td>
                  <td className="py-2 pr-4"><span className="text-route-red">Red</span></td>
                  <td className="py-2 pr-4 text-tc-cyan">4m 08s</td>
                  <td className="py-2"><span className="text-tc-amber">+2m LATE</span></td>
                </tr>
                <tr>
                  <td className="py-2 pr-4 text-tc-text">Westgate</td>
                  <td className="py-2 pr-4"><span className="text-route-green">Green</span></td>
                  <td className="py-2 pr-4 text-tc-text-dim">&mdash;</td>
                  <td className="py-2"><span className="text-tc-red">FAULT</span></td>
                </tr>
                <tr>
                  <td className="py-2 pr-4 text-tc-text">Lakeside</td>
                  <td className="py-2 pr-4"><span className="text-route-blue">Blue</span></td>
                  <td className="py-2 pr-4 text-tc-cyan">8m 30s</td>
                  <td className="py-2"><span className="text-tc-green">ON TIME</span></td>
                </tr>
                <tr>
                  <td className="py-2 pr-4 text-tc-text">Northfield</td>
                  <td className="py-2 pr-4"><span className="text-route-orange">Orange</span></td>
                  <td className="py-2 pr-4 text-tc-text-dim">&mdash;</td>
                  <td className="py-2"><span className="text-tc-text-dim">OFFLINE</span></td>
                </tr>
              </tbody>
            </table>
          </div>
        </Section>

        {/* ─────────────── LIVE FEED (fake terminal) ─────────────── */}
        <Section title="System Log">
          <div className="panel scanline bg-tc-base font-mono text-xs leading-5 overflow-x-auto">
            <div className="text-tc-text-dim">14:32:07.041</div>
            <div className="text-tc-green">[SYS] All lines reporting nominal</div>
            <div className="text-tc-text-dim mt-1">14:32:12.889</div>
            <div className="text-tc-cyan">[TRK] Train 2814 cleared CENTRAL platform 2 &rarr; UNION</div>
            <div className="text-tc-text-dim mt-1">14:32:18.204</div>
            <div className="text-tc-amber">[DLY] WESTGATE queue depth 3 — hold 45s</div>
            <div className="text-tc-text-dim mt-1">14:32:24.510</div>
            <div className="text-tc-red">[ALT] Signal S-4022 WEST unreachable — fallback manual</div>
            <div className="text-tc-text-dim mt-1">14:32:31.773</div>
            <div className="text-tc-text-muted">[HBT] Heartbeat OK — 247 active vehicles</div>
            <div className="mt-2 text-tc-green animate-pulse">_ </div>
          </div>
        </Section>

        {/* ─────────────── GLOWS ─────────────── */}
        <Section title="Glow Effects">
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            <div className="panel border-tc-green/30 glow-green text-center">
              <span className="text-tc-green text-sm font-mono">glow-green</span>
            </div>
            <div className="panel border-tc-cyan/30 glow-cyan text-center">
              <span className="text-tc-cyan text-sm font-mono">glow-cyan</span>
            </div>
            <div className="panel border-tc-amber/30 glow-amber text-center">
              <span className="text-tc-amber text-sm font-mono">glow-amber</span>
            </div>
            <div className="panel border-tc-red/30 glow-red text-center">
              <span className="text-tc-red text-sm font-mono">glow-red</span>
            </div>
          </div>
        </Section>

        {/* ─────────────── TEXT GLOW ─────────────── */}
        <Section title="Text Glow">
          <div className="panel bg-tc-base space-y-2 font-mono text-lg">
            <p className="text-tc-green text-glow-green">SYSTEM OPERATIONAL</p>
            <p className="text-tc-cyan text-glow-cyan">TRACKING 247 VEHICLES</p>
            <p className="text-tc-amber text-glow-amber">MINOR DELAYS DETECTED</p>
          </div>
        </Section>

        {/* ─────────────── ROUTE MAP LEGEND ─────────────── */}
        <Section title="Route Map Legend">
          <div className="panel inline-flex flex-col gap-2">
            {[
              { color: "bg-route-yellow", label: "Downtown — Airport", code: "YLOW" },
              { color: "bg-route-orange", label: "Northfield — Eastport", code: "ORNG" },
              { color: "bg-route-red", label: "Northfield — Lakeside", code: "RED" },
              { color: "bg-route-green", label: "Eastport — Westgate", code: "GREN" },
              { color: "bg-route-blue", label: "Suburbia — Westgate", code: "BLUE" },
            ].map((r) => (
              <div key={r.code} className="flex items-center gap-3">
                <div className={`w-8 h-1 rounded-full ${r.color}`} />
                <span className="text-sm text-tc-text w-56">{r.label}</span>
                <span className="text-xs font-mono text-tc-text-dim">{r.code}</span>
              </div>
            ))}
          </div>
        </Section>

        {/* ─────────────── COMPOSITE EXAMPLE ─────────────── */}
        <Section title="Composite — Station Board">
          <div className="panel bg-tc-base scanline max-w-lg">
            <div className="flex items-center justify-between mb-3">
              <div className="flex items-center gap-2">
                <span className="status-dot bg-tc-green animate-pulse" />
                <span className="font-mono text-xs text-tc-green uppercase tracking-wider">
                  Central Station
                </span>
              </div>
              <span className="text-xs font-mono text-tc-text-dim">PLT 2</span>
            </div>
            <div className="space-y-2 font-mono text-sm">
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <div className="w-6 h-0.5 rounded-full bg-route-yellow" />
                  <span className="text-tc-text">Airport</span>
                </div>
                <span className="text-tc-cyan text-glow-cyan">1m</span>
              </div>
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <div className="w-6 h-0.5 rounded-full bg-route-red" />
                  <span className="text-tc-text">Lakeside</span>
                </div>
                <span className="text-tc-cyan">4m</span>
              </div>
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <div className="w-6 h-0.5 rounded-full bg-route-blue" />
                  <span className="text-tc-text">Westgate</span>
                </div>
                <span className="text-tc-amber text-glow-amber">9m</span>
              </div>
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <div className="w-6 h-0.5 rounded-full bg-route-green" />
                  <span className="text-tc-text">Eastport</span>
                </div>
                <span className="text-tc-text-muted">12m</span>
              </div>
            </div>
            <div className="mt-3 pt-2 border-t border-tc-border text-xs font-mono text-tc-text-dim">
              Last update 14:32:07 &mdash; Refresh 15s
            </div>
          </div>
        </Section>

      </main>

      {/* Footer */}
      <footer className="border-t border-tc-border py-6 text-center">
        <p className="text-xs font-mono text-tc-text-dim">
          Control Center Design System &mdash; Transit Control Center
        </p>
      </footer>
    </div>
  );
}
