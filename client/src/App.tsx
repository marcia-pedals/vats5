import { useState, useMemo, ChangeEvent, MouseEvent } from 'react'
import { useQuery } from '@tanstack/react-query'
import { forceSimulation, forceManyBody, forceX, forceY, SimulationNodeDatum } from 'd3-force'
import Slider from 'rc-slider'
import 'rc-slider/assets/index.css'
import './App.css'

interface GtfsStop {
  stop_id: string
  stop_name: string
  stop_lat: number
  stop_lon: number
}

interface Stop {
  gtfs_stop: GtfsStop
  system_stop?: boolean
  intermediate_stop?: boolean
}

interface Step {
  origin_stop: number
  destination_stop: number
  origin_time: number
  destination_time: number
  trip_description: string
  system_step?: boolean
}

interface Path {
  steps: Step[]
}

interface Visualization {
  stops: [number, Stop][]
  adjacent: [number, Path[][]][]
}

interface NodeData extends SimulationNodeDatum {
  id: number
  stop: GtfsStop
  targetX: number
  targetY: number
}

interface Position {
  x: number
  y: number
}

interface EdgeData {
  from: number
  to: number
  paths: { originId: number; path: Path }[]
  allSystemSteps: boolean
}

interface SelectedEdge {
  from: number
  to: number
  fromStop: GtfsStop
  toStop: GtfsStop
  paths: { originId: number; path: Path }[]
}

function App() {
  const [selectedFile, setSelectedFile] = useState<string | null>(null)
  const [hoveredStop, setHoveredStop] = useState<GtfsStop | null>(null)
  const [hoveredEdge, setHoveredEdge] = useState<{ from: GtfsStop; to: GtfsStop } | null>(null)
  const [selectedEdge, setSelectedEdge] = useState<SelectedEdge | null>(null)
  const [mousePos, setMousePos] = useState<Position>({ x: 0, y: 0 })
  const [forceSimulationEnabled, setForceSimulationEnabled] = useState(true)
  const [hoveredRoute, setHoveredRoute] = useState<number[] | null>(null)
  const [originTimeMin, setOriginTimeMin] = useState(21600) // 6:00 in seconds from midnight
  const [originTimeMax, setOriginTimeMax] = useState(79200) // 22:00 in seconds from midnight

  const padding = 20
  const svgWidth = 800
  const svgHeight = 600

  // Fetch list of visualization files
  const { data: visualizationFiles = [], error: filesError } = useQuery({
    queryKey: ['visualizationFiles'],
    queryFn: async () => {
      const res = await fetch('/api/visualizations')
      return res.json() as Promise<string[]>
    },
  })

  // Derive effective selected file - use explicit selection or default to last file
  const effectiveSelectedFile = selectedFile ?? (visualizationFiles.length > 0 ? visualizationFiles[visualizationFiles.length - 1] : null)

  // Fetch selected visualization
  const { data: visualization, error: visualizationError } = useQuery({
    queryKey: ['visualization', effectiveSelectedFile],
    queryFn: async () => {
      const res = await fetch(`/api/visualizations/${effectiveSelectedFile}`)
      return res.json() as Promise<Visualization>
    },
    enabled: effectiveSelectedFile !== null,
  })

  const error = filesError || visualizationError

  // Compute node positions with force simulation
  const { stopsById, systemStopIds, nodes, nodePositions, edges, getStopPosition } = useMemo(() => {
    if (!visualization) {
      return {
        stopsById: new Map<number, GtfsStop>(),
        systemStopIds: new Set<number>(),
        nodes: [] as NodeData[],
        nodePositions: new Map<number, Position>(),
        edges: [] as EdgeData[],
        getStopPosition: () => null as Position | null
      }
    }

    const stopsById = new Map<number, GtfsStop>(visualization.stops.map(([id, stop]) => [id, stop.gtfs_stop]))
    const systemStopIds = new Set<number>(visualization.stops.filter(([, stop]) => stop.system_stop).map(([id]) => id))
    const graphStops = visualization.stops.filter(([, stop]) => stop.system_stop || stop.intermediate_stop)
    const stops: [number, GtfsStop][] = graphStops.map(([id, stop]) => [id, stop.gtfs_stop])

    if (stops.length === 0) {
      return {
        stopsById,
        systemStopIds,
        nodes: [] as NodeData[],
        nodePositions: new Map<number, Position>(),
        edges: [] as EdgeData[],
        getStopPosition: () => null as Position | null
      }
    }

    const lats = stops.map(([, s]) => s.stop_lat)
    const lons = stops.map(([, s]) => s.stop_lon)

    const minLat = Math.min(...lats)
    const maxLat = Math.max(...lats)
    const minLon = Math.min(...lons)
    const maxLon = Math.max(...lons)

    const latRange = maxLat - minLat || 1
    const lonRange = maxLon - minLon || 1

    const mapToSvg = (lat: number, lon: number): Position => {
      const x = padding + ((lon - minLon) / lonRange) * (svgWidth - 2 * padding)
      const y = padding + ((maxLat - lat) / latRange) * (svgHeight - 2 * padding)
      return { x, y }
    }

    // Create nodes with initial geographic positions
    const nodes: NodeData[] = stops.map(([id, stop]) => {
      const { x, y } = mapToSvg(stop.stop_lat, stop.stop_lon)
      return { id, stop, x, y, targetX: x, targetY: y }
    })

    // Run force simulation if enabled
    let nodePositions: Map<number, Position>
    if (forceSimulationEnabled) {
      const simulation = forceSimulation(nodes)
        .force('charge', forceManyBody().strength(-100))
        .force('x', forceX<NodeData>(d => d.targetX).strength(0.5))
        .force('y', forceY<NodeData>(d => d.targetY).strength(0.5))
        .stop()

      // Run simulation to completion
      for (let i = 0; i < 300; i++) {
        simulation.tick()
      }

      // Rescale to fit within SVG bounds
      const simXs = nodes.map(n => n.x!)
      const simYs = nodes.map(n => n.y!)
      const simMinX = Math.min(...simXs)
      const simMaxX = Math.max(...simXs)
      const simMinY = Math.min(...simYs)
      const simMaxY = Math.max(...simYs)
      const simRangeX = simMaxX - simMinX || 1
      const simRangeY = simMaxY - simMinY || 1

      nodePositions = new Map(nodes.map(n => [n.id, {
        x: padding + ((n.x! - simMinX) / simRangeX) * (svgWidth - 2 * padding),
        y: padding + ((n.y! - simMinY) / simRangeY) * (svgHeight - 2 * padding),
      }]))
    } else {
      // Use geographic positions directly without force simulation
      nodePositions = new Map(nodes.map(n => [n.id, {
        x: n.targetX,
        y: n.targetY,
      }]))
    }

    // Build edges and collect paths for each edge (filtered by origin time)
    const edgeMap = new Map<string, EdgeData>()
    for (const [originId, pathGroups] of visualization.adjacent) {
      for (const pathGroup of pathGroups) {
        for (const path of pathGroup) {
          if (path.steps.length === 0) continue
          // Filter by origin time of first step
          const firstStepOriginTime = path.steps[0].origin_time
          if (firstStepOriginTime < originTimeMin || firstStepOriginTime > originTimeMax) continue

          const lastStep = path.steps[path.steps.length - 1]
          const destId = lastStep.destination_stop
          if (stopsById.has(originId) && stopsById.has(destId)) {
            const key = originId < destId ? `${originId}-${destId}` : `${destId}-${originId}`
            if (!edgeMap.has(key)) {
              const [from, to] = key.split('-').map(Number)
              edgeMap.set(key, { from, to, paths: [], allSystemSteps: true })
            }
            edgeMap.get(key)!.paths.push({ originId, path })
            // Check if all steps in this path are system steps
            const allStepsSystem = path.steps.every(step => step.system_step)
            if (!allStepsSystem) {
              edgeMap.get(key)!.allSystemSteps = false
            }
          }
        }
      }
    }
    const edges = [...edgeMap.values()]

    // Function to get position for any stop (including those not in reduced graph)
    const getStopPosition = (stopId: number): Position | null => {
      // First check if it's in nodePositions (reduced graph nodes)
      if (nodePositions.has(stopId)) {
        return nodePositions.get(stopId)!
      }
      // Otherwise compute position from lat/lon using mapToSvg
      const stop = stopsById.get(stopId)
      if (stop) {
        return mapToSvg(stop.stop_lat, stop.stop_lon)
      }
      return null
    }

    return { stopsById, systemStopIds, nodes, nodePositions, edges, getStopPosition }
  }, [visualization, forceSimulationEnabled, originTimeMin, originTimeMax])

  const formatTimeForInput = (seconds: number): string => {
    const h = Math.floor(seconds / 3600)
    const m = Math.floor((seconds % 3600) / 60)
    return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}`
  }

  const currentIndex = effectiveSelectedFile ? visualizationFiles.indexOf(effectiveSelectedFile) : -1
  const canGoPrev = currentIndex > 0
  const canGoNext = currentIndex < visualizationFiles.length - 1

  const isLoading = !visualization || nodes.length === 0

  return (
    <div style={{ position: 'relative' }}>
      {/* File selector bar */}
      <div style={{
        display: 'flex',
        alignItems: 'center',
        gap: '10px',
        marginBottom: '10px',
        padding: '8px',
        background: '#f5f5f5',
        borderRadius: '4px',
      }}>
        <button
          onClick={() => canGoPrev && setSelectedFile(visualizationFiles[currentIndex - 1])}
          disabled={!canGoPrev}
          style={{
            padding: '4px 12px',
            cursor: canGoPrev ? 'pointer' : 'not-allowed',
            opacity: canGoPrev ? 1 : 0.5,
          }}
        >
          ←
        </button>
        <select
          value={effectiveSelectedFile || ''}
          onChange={(e: ChangeEvent<HTMLSelectElement>) => setSelectedFile(e.target.value)}
          style={{
            padding: '4px 8px',
            minWidth: '180px',
          }}
        >
          {visualizationFiles.map(file => (
            <option key={file} value={file}>
              {file.replace('.json', '')}
            </option>
          ))}
        </select>
        <button
          onClick={() => canGoNext && setSelectedFile(visualizationFiles[currentIndex + 1])}
          disabled={!canGoNext}
          style={{
            padding: '4px 12px',
            cursor: canGoNext ? 'pointer' : 'not-allowed',
            opacity: canGoNext ? 1 : 0.5,
          }}
        >
          →
        </button>
        <span style={{ color: '#666', fontSize: '14px' }}>
          {currentIndex + 1} / {visualizationFiles.length}
        </span>
        <label style={{ marginLeft: '20px', display: 'flex', alignItems: 'center', cursor: 'pointer' }}>
          <input
            type="checkbox"
            checked={forceSimulationEnabled}
            onChange={(e: ChangeEvent<HTMLInputElement>) => setForceSimulationEnabled(e.target.checked)}
          />
          {' '}Spread stops
        </label>
      </div>

      {/* Main content */}
      <div>
        <div style={{ marginBottom: '10px', display: 'flex', alignItems: 'center', gap: '10px' }}>
          <span style={{ whiteSpace: 'nowrap' }}>Origin time: {formatTimeForInput(originTimeMin)} - {formatTimeForInput(originTimeMax)}</span>
          <Slider
            range
            min={0}
            max={129600}
            step={300}
            value={[originTimeMin, originTimeMax]}
            onChange={(value) => {
              const [min, max] = value as [number, number]
              setOriginTimeMin(min)
              setOriginTimeMax(max)
            }}
            style={{ flex: 1 }}
          />
        </div>
        <div style={{ display: 'flex', gap: '20px' }}>
      {isLoading || error ? (
        <div style={{
          width: svgWidth,
          height: svgHeight,
          border: '1px solid black',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          color: error ? 'red' : '#666',
        }}>
          {error ? `Error: ${error.message}` : 'Loading...'}
        </div>
      ) : (
        <svg width={svgWidth} height={svgHeight} style={{ border: '1px solid black' }}>
          {edges.map((edge, i) => {
            const fromStop = stopsById.get(edge.from)!
            const toStop = stopsById.get(edge.to)!
            const from = nodePositions.get(edge.from)!
            const to = nodePositions.get(edge.to)!
            const isSelected = selectedEdge?.from === edge.from && selectedEdge?.to === edge.to
            return (
              <line
                key={i}
                x1={from.x}
                y1={from.y}
                x2={to.x}
                y2={to.y}
                stroke={isSelected ? 'orange' : 'gray'}
                strokeWidth={isSelected ? 3 : 2}
                strokeDasharray={edge.allSystemSteps ? "5,5" : undefined}
                style={{ cursor: 'pointer' }}
                onMouseEnter={(e: MouseEvent<SVGLineElement>) => {
                  setHoveredEdge({ from: fromStop, to: toStop })
                  setMousePos({ x: e.clientX, y: e.clientY })
                }}
                onMouseMove={(e: MouseEvent<SVGLineElement>) => {
                  setMousePos({ x: e.clientX, y: e.clientY })
                }}
                onMouseLeave={() => setHoveredEdge(null)}
                onClick={() => setSelectedEdge({ from: edge.from, to: edge.to, fromStop, toStop, paths: edge.paths })}
              />
            )
          })}
          {nodes.map((node) => {
            const pos = nodePositions.get(node.id)!
            const isSystemStop = systemStopIds.has(node.id)
            return (
              <circle
                key={node.id}
                cx={pos.x}
                cy={pos.y}
                r={5}
                fill={isSystemStop ? "blue" : "#444"}
                style={{ cursor: 'pointer' }}
                onMouseEnter={(e: MouseEvent<SVGCircleElement>) => {
                  setHoveredStop(node.stop)
                  setMousePos({ x: e.clientX, y: e.clientY })
                }}
                onMouseMove={(e: MouseEvent<SVGCircleElement>) => {
                  setMousePos({ x: e.clientX, y: e.clientY })
                }}
                onMouseLeave={() => setHoveredStop(null)}
              />
            )
          })}
          {hoveredRoute && hoveredRoute.length > 1 && (() => {
            const points = hoveredRoute
              .map(stopId => getStopPosition(stopId))
              .filter((pos): pos is Position => pos !== null)
              .map(pos => `${pos.x},${pos.y}`)
              .join(' ')
            return (
              <polyline
                points={points}
                fill="none"
                stroke="red"
                strokeWidth={3}
                strokeLinecap="round"
                strokeLinejoin="round"
                pointerEvents="none"
              />
            )
          })()}
        </svg>
      )}
      <div style={{
        width: '400px',
        padding: '10px',
        border: '1px solid #ccc',
        borderRadius: '4px',
        maxHeight: '580px',
        overflowY: 'auto',
        textAlign: 'left'
      }}>
        {selectedEdge ? (
          <div style={{ fontSize: '14px' }}>
            {[
              { from: selectedEdge.from, to: selectedEdge.to, fromStop: selectedEdge.fromStop, toStop: selectedEdge.toStop },
              { from: selectedEdge.to, to: selectedEdge.from, fromStop: selectedEdge.toStop, toStop: selectedEdge.fromStop }
            ].map(({ from, to, fromStop, toStop }) => {
              const directionPaths = selectedEdge.paths.filter(p => p.originId === from)
              return (
                <div key={`${from}-${to}`} style={{ marginBottom: '15px' }}>
                  <h4 style={{ margin: '0 0 8px 0' }}>{fromStop.stop_name} → {toStop.stop_name}</h4>
                  {directionPaths.length === 0 ? (
                    <div style={{ color: '#888', marginLeft: '10px' }}>No paths</div>
                  ) : (() => {
                    const formatTime = (seconds: number): string => {
                      const h = Math.floor(seconds / 3600)
                      const m = Math.floor((seconds % 3600) / 60)
                      return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}`
                    }
                    const collapseSteps = (steps: Step[]) => steps.reduce((acc, step) => {
                      const last = acc[acc.length - 1]
                      if (last && last.trip_description === step.trip_description) {
                        last.destination_stop = step.destination_stop
                      } else {
                        acc.push({ ...step })
                      }
                      return acc
                    }, [] as Step[])
                    const getSignature = (segments: Step[]) => segments.map(s =>
                      `${s.trip_description}:${s.origin_stop}:${s.destination_stop}`
                    ).join('|')

                    // Group paths by their signature
                    const groups = new Map<string, { segments: Step[]; steps: Step[]; times: { start: number; end: number }[] }>()
                    for (const { path } of directionPaths) {
                      const segments = collapseSteps(path.steps)
                      const sig = getSignature(segments)
                      if (!groups.has(sig)) {
                        groups.set(sig, { segments, steps: path.steps, times: [] })
                      }
                      const firstStep = path.steps[0]
                      const lastStep = path.steps[path.steps.length - 1]
                      groups.get(sig)!.times.push({
                        start: firstStep.origin_time,
                        end: lastStep.destination_time
                      })
                    }

                    return [...groups.values()].map((group, groupIndex) => {
                      // Build the list of stop IDs for this route from uncollapsed steps
                      const routeStops: number[] = []
                      for (const step of group.steps) {
                        if (routeStops.length === 0) {
                          routeStops.push(step.origin_stop)
                        }
                        routeStops.push(step.destination_stop)
                      }
                      return (
                      <div key={groupIndex} style={{
                        marginBottom: '8px',
                        padding: '6px',
                        background: '#f5f5f5',
                        borderRadius: '4px',
                        cursor: 'pointer'
                      }}
                      onMouseEnter={() => setHoveredRoute(routeStops)}
                      onMouseLeave={() => setHoveredRoute(null)}
                      >
                        {group.segments.map((segment, segmentIndex) => {
                          const tripDesc = segment.trip_description.startsWith('Walk from') ? 'Walk' : segment.trip_description
                          return (
                            <div key={segmentIndex} style={{ fontSize: '12px', color: '#555' }}>
                              {tripDesc}: {stopsById.get(segment.origin_stop)?.stop_name || segment.origin_stop} → {stopsById.get(segment.destination_stop)?.stop_name || segment.destination_stop}
                            </div>
                          )
                        })}
                        <div style={{ marginTop: '4px', fontSize: '12px', fontFamily: 'monospace', display: 'flex', flexWrap: 'wrap', gap: '4px 8px' }}>
                          {group.times.map((t, i) => (
                            <span key={i} style={{ whiteSpace: 'nowrap' }}>
                              {formatTime(t.start)}-{formatTime(t.end)}
                            </span>
                          ))}
                        </div>
                      </div>
                    )})
                  })()}
                </div>
              )
            })}
          </div>
        ) : (
          <div style={{ color: '#888' }}>Click an edge to select</div>
        )}
        </div>
        </div>
      </div>
      {(hoveredStop || hoveredEdge) && (
        <div style={{
          position: 'fixed',
          left: mousePos.x + 10,
          top: mousePos.y + 10,
          background: 'black',
          color: 'white',
          padding: '4px 8px',
          borderRadius: '4px',
          fontSize: '14px',
          pointerEvents: 'none',
        }}>
          {hoveredStop && hoveredStop.stop_name}
          {hoveredEdge && `${hoveredEdge.from.stop_name} ↔ ${hoveredEdge.to.stop_name}`}
        </div>
      )}
    </div>
  )
}

export default App
