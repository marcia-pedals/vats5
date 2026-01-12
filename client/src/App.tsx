import { useState, useMemo, useEffect, useRef, useCallback, ChangeEvent, MouseEvent } from 'react'
import { useQuery, useQueryClient, keepPreviousData } from '@tanstack/react-query'
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


function App() {
  const queryClient = useQueryClient()
  const [selectedFile, setSelectedFile] = useState<string | null>(null)
  const [hoveredStop, setHoveredStop] = useState<GtfsStop | null>(null)
  const [selectedStop, setSelectedStop] = useState<{ id: number; stop: GtfsStop } | null>(null)
  const [hoveredEdge, setHoveredEdge] = useState<{ from: GtfsStop; to: GtfsStop; tripDescriptions: string[] } | null>(null)
  const [mousePos, setMousePos] = useState<Position>({ x: 0, y: 0 })
  const [hoveredRoute, setHoveredRoute] = useState<number[] | null>(null)
  const [originTimeMin, setOriginTimeMin] = useState(21600) // 6:00 in seconds from midnight
  const [originTimeMax, setOriginTimeMax] = useState(79200) // 22:00 in seconds from midnight

  // Pan and zoom state
  const [pan, setPan] = useState<Position>({ x: 0, y: 0 })
  const [zoom, setZoom] = useState(1)
  const [isPanning, setIsPanning] = useState(false)
  const [panStart, setPanStart] = useState<Position>({ x: 0, y: 0 })
  const svgRef = useRef<SVGSVGElement>(null)
  const panRef = useRef(pan)
  const zoomRef = useRef(zoom)
  panRef.current = pan
  zoomRef.current = zoom

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
    staleTime: Infinity,
  })

  // Derive effective selected file - use explicit selection or default to first file
  const effectiveSelectedFile = selectedFile ?? (visualizationFiles.length > 0 ? visualizationFiles[0] : null)

  // Fetch selected visualization
  const { data: visualization, error: visualizationError, isFetching } = useQuery({
    queryKey: ['visualization', effectiveSelectedFile],
    queryFn: async () => {
      const res = await fetch(`/api/visualizations/${effectiveSelectedFile}`)
      return res.json() as Promise<Visualization>
    },
    enabled: effectiveSelectedFile !== null,
    placeholderData: keepPreviousData,
    staleTime: Infinity,
  })

  const error = filesError || visualizationError

  // Prefetch all visualizations in the background
  useEffect(() => {
    for (const file of visualizationFiles) {
      queryClient.prefetchQuery({
        queryKey: ['visualization', file],
        queryFn: async () => {
          const res = await fetch(`/api/visualizations/${file}`)
          return res.json() as Promise<Visualization>
        },
        staleTime: Infinity,
      })
    }
  }, [visualizationFiles, queryClient])

  // Compute node positions (geographic only, no force simulation)
  const { stopsById, systemStopIds, nodes, nodePositions, edges, getStopPosition } = useMemo(() => {
    if (!visualization) {
      return {
        stopsById: new Map<number, GtfsStop>(),
        systemStopIds: new Set<number>(),
        nodes: [] as { id: number; stop: GtfsStop }[],
        nodePositions: new Map<number, Position>(),
        edges: [] as EdgeData[],
        getStopPosition: () => null as Position | null
      }
    }

    const stopsById = new Map<number, GtfsStop>(visualization.stops.map(([id, stop]) => [id, stop.gtfs_stop]))
    const systemStopIds = new Set<number>(visualization.stops.filter(([, stop]) => stop.system_stop).map(([id]) => id))

    // Collect all stop IDs that appear in any path
    const stopsInPaths = new Set<number>()
    for (const [originId, pathGroups] of visualization.adjacent) {
      for (const pathGroup of pathGroups) {
        for (const path of pathGroup) {
          for (const step of path.steps) {
            stopsInPaths.add(step.origin_stop)
            stopsInPaths.add(step.destination_stop)
          }
        }
      }
    }

    const graphStops = visualization.stops.filter(([id]) => stopsInPaths.has(id))
    const stops: [number, GtfsStop][] = graphStops.map(([id, stop]) => [id, stop.gtfs_stop])

    if (stops.length === 0) {
      return {
        stopsById,
        systemStopIds,
        nodes: [] as { id: number; stop: GtfsStop }[],
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

    // Create nodes with geographic positions
    const nodes = stops.map(([id, stop]) => {
      return { id, stop }
    })

    // Use geographic positions directly
    const nodePositions = new Map(stops.map(([id, stop]) => {
      const pos = mapToSvg(stop.stop_lat, stop.stop_lon)
      return [id, pos]
    }))

    // Build edges for each individual step (filtered by origin time)
    const edgeMap = new Map<string, EdgeData>()
    for (const [originId, pathGroups] of visualization.adjacent) {
      for (const pathGroup of pathGroups) {
        for (const path of pathGroup) {
          if (path.steps.length === 0) continue
          // Filter by origin time of first step
          const firstStepOriginTime = path.steps[0].origin_time
          if (firstStepOriginTime < originTimeMin || firstStepOriginTime > originTimeMax) continue

          for (const step of path.steps) {
            const fromId = step.origin_stop
            const toId = step.destination_stop
            if (stopsById.has(fromId) && stopsById.has(toId)) {
              const key = fromId < toId ? `${fromId}-${toId}` : `${toId}-${fromId}`
              if (!edgeMap.has(key)) {
                const [from, to] = key.split('-').map(Number)
                edgeMap.set(key, { from, to, paths: [], allSystemSteps: true })
              }
              edgeMap.get(key)!.paths.push({ originId, path })
              if (!step.system_step) {
                edgeMap.get(key)!.allSystemSteps = false
              }
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
  }, [visualization, originTimeMin, originTimeMax])

  // Compute highlighted stops, edges, and paths by destination when a stop is selected
  const { highlightedStops, highlightedEdges, pathsByDestination } = useMemo(() => {
    if (!selectedStop || !visualization) {
      return { highlightedStops: null, highlightedEdges: null, pathsByDestination: null }
    }

    const highlightedStops = new Set<number>()
    const highlightedEdges = new Set<string>()
    // Map from destination stop ID to list of paths
    const pathsByDest = new Map<number, Path[]>()

    for (const [, pathGroups] of visualization.adjacent) {
      for (const pathGroup of pathGroups) {
        for (const path of pathGroup) {
          if (path.steps.length === 0) continue

          // Filter by origin time
          const firstStepOriginTime = path.steps[0].origin_time
          if (firstStepOriginTime < originTimeMin || firstStepOriginTime > originTimeMax) continue

          const firstStop = path.steps[0].origin_stop
          const lastStop = path.steps[path.steps.length - 1].destination_stop

          // Check if this path starts or ends at the selected stop
          if (firstStop === selectedStop.id) {
            // Add all stops and edges from this path
            for (const step of path.steps) {
              highlightedStops.add(step.origin_stop)
              highlightedStops.add(step.destination_stop)
              const edgeKey = step.origin_stop < step.destination_stop
                ? `${step.origin_stop}-${step.destination_stop}`
                : `${step.destination_stop}-${step.origin_stop}`
              highlightedEdges.add(edgeKey)
            }
            // Group by destination
            if (!pathsByDest.has(lastStop)) {
              pathsByDest.set(lastStop, [])
            }
            pathsByDest.get(lastStop)!.push(path)
          } else if (lastStop === selectedStop.id) {
            // Add all stops and edges from this path
            for (const step of path.steps) {
              highlightedStops.add(step.origin_stop)
              highlightedStops.add(step.destination_stop)
              const edgeKey = step.origin_stop < step.destination_stop
                ? `${step.origin_stop}-${step.destination_stop}`
                : `${step.destination_stop}-${step.origin_stop}`
              highlightedEdges.add(edgeKey)
            }
          }
        }
      }
    }

    return { highlightedStops, highlightedEdges, pathsByDestination: pathsByDest }
  }, [selectedStop, visualization, originTimeMin, originTimeMax])

  const formatTimeForInput = (seconds: number): string => {
    const h = Math.floor(seconds / 3600)
    const m = Math.floor((seconds % 3600) / 60)
    return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}`
  }

  const currentIndex = effectiveSelectedFile ? visualizationFiles.indexOf(effectiveSelectedFile) : -1
  const canGoPrev = currentIndex > 0
  const canGoNext = currentIndex < visualizationFiles.length - 1

  // Pan and zoom handlers
  useEffect(() => {
    const svg = svgRef.current
    if (!svg) return

    const handleWheel = (e: globalThis.WheelEvent) => {
      e.preventDefault()
      const rect = svg.getBoundingClientRect()

      const mouseX = e.clientX - rect.left
      const mouseY = e.clientY - rect.top

      const zoomFactor = e.deltaY > 0 ? 0.92 : 1.08
      const oldZoom = zoomRef.current
      const newZoom = Math.min(Math.max(oldZoom * zoomFactor, 0.5), 10)

      const oldPan = panRef.current
      const newPanX = mouseX - (mouseX - oldPan.x) * (newZoom / oldZoom)
      const newPanY = mouseY - (mouseY - oldPan.y) * (newZoom / oldZoom)

      setZoom(newZoom)
      setPan({ x: newPanX, y: newPanY })
    }

    svg.addEventListener('wheel', handleWheel, { passive: false })
    return () => svg.removeEventListener('wheel', handleWheel)
  }, [])

  const handleMouseDown = useCallback((e: MouseEvent<SVGSVGElement>) => {
    if (e.button === 0) {
      setIsPanning(true)
      setPanStart({ x: e.clientX - pan.x, y: e.clientY - pan.y })
    }
  }, [pan])

  const handleMouseMove = useCallback((e: MouseEvent<SVGSVGElement>) => {
    if (isPanning) {
      setPan({ x: e.clientX - panStart.x, y: e.clientY - panStart.y })
    }
  }, [isPanning, panStart])

  const handleMouseUp = useCallback(() => {
    setIsPanning(false)
  }, [])

  const handleMouseLeave = useCallback(() => {
    setIsPanning(false)
  }, [])

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
      </div>

      {/* Main content */}
      <div>
        <div style={{ marginBottom: '10px', display: 'flex', alignItems: 'center', gap: '10px' }}>
          <span style={{ whiteSpace: 'nowrap' }}>Departure: {formatTimeForInput(originTimeMin)} - {formatTimeForInput(originTimeMax)}</span>
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
      <div style={{ position: 'relative' }}>
        <svg
          ref={svgRef}
          width={svgWidth}
          height={svgHeight}
          style={{ border: '1px solid black', cursor: isPanning ? 'grabbing' : 'grab' }}
          onMouseDown={handleMouseDown}
          onMouseMove={handleMouseMove}
          onMouseUp={handleMouseUp}
          onMouseLeave={handleMouseLeave}
        >
          <g transform={`translate(${pan.x}, ${pan.y}) scale(${zoom})`}>
          {edges.map((edge, i) => {
            const fromStop = stopsById.get(edge.from)!
            const toStop = stopsById.get(edge.to)!
            const from = nodePositions.get(edge.from)!
            const to = nodePositions.get(edge.to)!
            const edgeKey = `${edge.from}-${edge.to}`
            const isDimmed = highlightedEdges && !highlightedEdges.has(edgeKey)
            return (
              <g key={i} opacity={isDimmed ? 0.2 : 1}>
                <line
                  x1={from.x}
                  y1={from.y}
                  x2={to.x}
                  y2={to.y}
                  stroke="gray"
                  strokeWidth={2 / zoom}
                  strokeDasharray={edge.allSystemSteps ? "5,5" : undefined}
                  pointerEvents="none"
                />
                <line
                  x1={from.x}
                  y1={from.y}
                  x2={to.x}
                  y2={to.y}
                  stroke="transparent"
                  strokeWidth={12 / zoom}
                  style={{ cursor: 'pointer' }}
                  onMouseEnter={(e: MouseEvent<SVGLineElement>) => {
                    const tripDescriptions = [...new Set(
                      edge.paths.flatMap(p => p.path.steps
                        .filter(s =>
                          (s.origin_stop === edge.from && s.destination_stop === edge.to) ||
                          (s.origin_stop === edge.to && s.destination_stop === edge.from)
                        )
                        .map(s => s.trip_description)
                      )
                    )]
                    setHoveredEdge({ from: fromStop, to: toStop, tripDescriptions })
                    setMousePos({ x: e.clientX, y: e.clientY })
                  }}
                  onMouseMove={(e: MouseEvent<SVGLineElement>) => {
                    setMousePos({ x: e.clientX, y: e.clientY })
                  }}
                  onMouseLeave={() => setHoveredEdge(null)}
                />
              </g>
            )
          })}
          {nodes.map((node) => {
            const pos = nodePositions.get(node.id)!
            const isSystemStop = systemStopIds.has(node.id)
            const isDimmed = highlightedStops && !highlightedStops.has(node.id)
            return (
              <circle
                key={node.id}
                cx={pos.x}
                cy={pos.y}
                r={(isSystemStop ? 5 : 2) / zoom}
                fill={isSystemStop ? "blue" : "#444"}
                opacity={isDimmed ? 0.2 : 1}
                style={{ cursor: 'pointer' }}
                onMouseEnter={(e: MouseEvent<SVGCircleElement>) => {
                  setHoveredStop(node.stop)
                  setMousePos({ x: e.clientX, y: e.clientY })
                }}
                onMouseMove={(e: MouseEvent<SVGCircleElement>) => {
                  setMousePos({ x: e.clientX, y: e.clientY })
                }}
                onMouseLeave={() => setHoveredStop(null)}
                onClick={() => setSelectedStop({ id: node.id, stop: node.stop })}
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
                strokeWidth={3 / zoom}
                strokeLinecap="round"
                strokeLinejoin="round"
                pointerEvents="none"
              />
            )
          })()}
          </g>
        </svg>
        {(isFetching || error) && (
          <div style={{
            position: 'absolute',
            top: 0,
            left: 0,
            width: svgWidth,
            height: svgHeight,
            background: 'rgba(255, 255, 255, 0.7)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            color: error ? 'red' : '#666',
            fontSize: '18px',
          }}>
            {error ? `Error: ${error.message}` : 'Loading...'}
          </div>
        )}
      </div>
      <div style={{
        width: '400px',
        padding: '10px',
        border: '1px solid #ccc',
        borderRadius: '4px',
        maxHeight: '580px',
        overflowY: 'auto',
        textAlign: 'left'
      }}>
        {selectedStop && pathsByDestination ? (
          <div style={{ fontSize: '14px' }}>
            <h3 style={{ margin: '0 0 12px 0' }}>{selectedStop.stop.stop_name}</h3>
            {[...pathsByDestination.entries()].map(([destId, paths]) => {
              const destStop = stopsById.get(destId)
              if (!destStop) return null

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
              for (const path of paths) {
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

              return (
                <div key={destId} style={{ marginBottom: '15px' }}>
                  <h4 style={{ margin: '0 0 8px 0' }}>→ {destStop.stop_name}</h4>
                  {[...groups.values()].map((group, groupIndex) => {
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
                    )
                  })}
                </div>
              )
            })}
          </div>
        ) : (
          <div style={{ color: '#888' }}>Click a stop to see paths</div>
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
          {hoveredEdge && (
            <div>
              <div>{hoveredEdge.from.stop_name} ↔ {hoveredEdge.to.stop_name}</div>
              {hoveredEdge.tripDescriptions.map((desc, i) => (
                <div key={i} style={{ fontSize: '12px', opacity: 0.9 }}>{desc}</div>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  )
}

export default App
