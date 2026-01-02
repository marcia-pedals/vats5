import { useState, useEffect, useMemo } from 'react'
import { forceSimulation, forceManyBody, forceX, forceY } from 'd3-force'
import './App.css'

function App() {
  const [visualization, setVisualization] = useState(null)
  const [error, setError] = useState(null)
  const [hoveredStop, setHoveredStop] = useState(null)
  const [hoveredEdge, setHoveredEdge] = useState(null)
  const [selectedEdge, setSelectedEdge] = useState(null)
  const [mousePos, setMousePos] = useState({ x: 0, y: 0 })
  const [forceSimulationEnabled, setForceSimulationEnabled] = useState(true)
  const [hoveredRoute, setHoveredRoute] = useState(null) // Array of stop IDs for the hovered route
  const [originTimeMin, setOriginTimeMin] = useState(21600) // 6:00 in seconds from midnight
  const [originTimeMax, setOriginTimeMax] = useState(79200) // 22:00 in seconds from midnight

  const padding = 20
  const svgWidth = 800
  const svgHeight = 600

  useEffect(() => {
    fetch('/api/visualization')
      .then(res => res.json())
      .then(setVisualization)
      .catch(err => setError('Error: ' + err.message))
  }, [])

  // Compute node positions with force simulation
  const { stopsById, systemStopIds, nodes, nodePositions, edges, getStopPosition } = useMemo(() => {
    if (!visualization) {
      return { stopsById: new Map(), systemStopIds: new Set(), nodes: [], nodePositions: new Map(), edges: [], getStopPosition: () => null }
    }

    const stopsById = new Map(visualization.stops.map(([id, stop]) => [id, stop.gtfs_stop]))
    const systemStopIds = new Set(visualization.stops.filter(([, stop]) => stop.system_stop).map(([id]) => id))
    const graphStops = visualization.stops.filter(([, stop]) => stop.system_stop || stop.intermediate_stop)
    const stops = graphStops.map(([id, stop]) => [id, stop.gtfs_stop])

    if (stops.length === 0) {
      return { stopsById, systemStopIds, nodes: [], nodePositions: new Map(), edges: [], getStopPosition: () => null }
    }

    const lats = stops.map(([, s]) => s.stop_lat)
    const lons = stops.map(([, s]) => s.stop_lon)

    const minLat = Math.min(...lats)
    const maxLat = Math.max(...lats)
    const minLon = Math.min(...lons)
    const maxLon = Math.max(...lons)

    const latRange = maxLat - minLat || 1
    const lonRange = maxLon - minLon || 1

    const mapToSvg = (lat, lon) => {
      const x = padding + ((lon - minLon) / lonRange) * (svgWidth - 2 * padding)
      const y = padding + ((maxLat - lat) / latRange) * (svgHeight - 2 * padding)
      return { x, y }
    }

    // Create nodes with initial geographic positions
    const nodes = stops.map(([id, stop]) => {
      const { x, y } = mapToSvg(stop.stop_lat, stop.stop_lon)
      return { id, stop, x, y, targetX: x, targetY: y }
    })

    // Run force simulation if enabled
    let nodePositions
    if (forceSimulationEnabled) {
      const simulation = forceSimulation(nodes)
        .force('charge', forceManyBody().strength(-100))
        .force('x', forceX(d => d.targetX).strength(0.5))
        .force('y', forceY(d => d.targetY).strength(0.5))
        .stop()

      // Run simulation to completion
      for (let i = 0; i < 300; i++) {
        simulation.tick()
      }

      // Rescale to fit within SVG bounds
      const simXs = nodes.map(n => n.x)
      const simYs = nodes.map(n => n.y)
      const simMinX = Math.min(...simXs)
      const simMaxX = Math.max(...simXs)
      const simMinY = Math.min(...simYs)
      const simMaxY = Math.max(...simYs)
      const simRangeX = simMaxX - simMinX || 1
      const simRangeY = simMaxY - simMinY || 1

      nodePositions = new Map(nodes.map(n => [n.id, {
        x: padding + ((n.x - simMinX) / simRangeX) * (svgWidth - 2 * padding),
        y: padding + ((n.y - simMinY) / simRangeY) * (svgHeight - 2 * padding),
      }]))
    } else {
      // Use geographic positions directly without force simulation
      nodePositions = new Map(nodes.map(n => [n.id, {
        x: n.targetX,
        y: n.targetY,
      }]))
    }

    // Build edges and collect paths for each edge (filtered by origin time)
    const edgeMap = new Map() // key -> { from, to, paths: [], allSystemSteps: bool }
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
            edgeMap.get(key).paths.push({ originId, path })
            // Check if all steps in this path are system steps
            const allStepsSystem = path.steps.every(step => step.system_step)
            if (!allStepsSystem) {
              edgeMap.get(key).allSystemSteps = false
            }
          }
        }
      }
    }
    const edges = [...edgeMap.values()]

    // Function to get position for any stop (including those not in reduced graph)
    const getStopPosition = (stopId) => {
      // First check if it's in nodePositions (reduced graph nodes)
      if (nodePositions.has(stopId)) {
        return nodePositions.get(stopId)
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

  if (error) {
    return <div>{error}</div>
  }

  if (!visualization || nodes.length === 0) {
    return <div>Loading...</div>
  }

  const formatTimeForInput = (seconds) => {
    const h = Math.floor(seconds / 3600)
    const m = Math.floor((seconds % 3600) / 60)
    return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}`
  }

  return (
    <div style={{ position: 'relative' }}>
      <label style={{ display: 'block', marginBottom: '10px' }}>
        <input
          type="checkbox"
          checked={forceSimulationEnabled}
          onChange={(e) => setForceSimulationEnabled(e.target.checked)}
        />
        {' '}Force simulation
      </label>
      <div style={{ marginBottom: '10px' }}>
        <div style={{ marginBottom: '5px' }}>
          <span>Origin time: {formatTimeForInput(originTimeMin)} - {formatTimeForInput(originTimeMax)}</span>
        </div>
        <div style={{ display: 'flex', alignItems: 'center', marginBottom: '5px' }}>
          <span style={{ width: '40px' }}>Min:</span>
          <input
            type="range"
            min={0}
            max={129600}
            step={300}
            value={originTimeMin}
            onChange={(e) => setOriginTimeMin(Math.min(Number(e.target.value), originTimeMax))}
            style={{ flex: 1 }}
          />
        </div>
        <div style={{ display: 'flex', alignItems: 'center' }}>
          <span style={{ width: '40px' }}>Max:</span>
          <input
            type="range"
            min={0}
            max={129600}
            step={300}
            value={originTimeMax}
            onChange={(e) => setOriginTimeMax(Math.max(Number(e.target.value), originTimeMin))}
            style={{ flex: 1 }}
          />
        </div>
      </div>
      <div style={{ display: 'flex', gap: '20px' }}>
      <svg width={svgWidth} height={svgHeight} style={{ border: '1px solid black' }}>
        {edges.map((edge, i) => {
          const fromStop = stopsById.get(edge.from)
          const toStop = stopsById.get(edge.to)
          const from = nodePositions.get(edge.from)
          const to = nodePositions.get(edge.to)
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
              onMouseEnter={(e) => {
                setHoveredEdge({ from: fromStop, to: toStop })
                setMousePos({ x: e.clientX, y: e.clientY })
              }}
              onMouseMove={(e) => {
                setMousePos({ x: e.clientX, y: e.clientY })
              }}
              onMouseLeave={() => setHoveredEdge(null)}
              onClick={() => setSelectedEdge({ from: edge.from, to: edge.to, fromStop, toStop, paths: edge.paths })}
            />
          )
        })}
        {nodes.map((node) => {
          const pos = nodePositions.get(node.id)
          const isSystemStop = systemStopIds.has(node.id)
          return (
            <circle
              key={node.id}
              cx={pos.x}
              cy={pos.y}
              r={5}
              fill={isSystemStop ? "blue" : "#444"}
              style={{ cursor: 'pointer' }}
              onMouseEnter={(e) => {
                setHoveredStop(node.stop)
                setMousePos({ x: e.clientX, y: e.clientY })
              }}
              onMouseMove={(e) => {
                setMousePos({ x: e.clientX, y: e.clientY })
              }}
              onMouseLeave={() => setHoveredStop(null)}
            />
          )
        })}
        {hoveredRoute && hoveredRoute.length > 1 && (() => {
          const points = hoveredRoute
            .map(stopId => getStopPosition(stopId))
            .filter(pos => pos)
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
                    const formatTime = (seconds) => {
                      const h = Math.floor(seconds / 3600)
                      const m = Math.floor((seconds % 3600) / 60)
                      return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}`
                    }
                    const collapseSteps = (steps) => steps.reduce((acc, step) => {
                      const last = acc[acc.length - 1]
                      if (last && last.trip_description === step.trip_description) {
                        last.destination_stop = step.destination_stop
                      } else {
                        acc.push({ ...step })
                      }
                      return acc
                    }, [])
                    const getSignature = (segments) => segments.map(s =>
                      `${s.trip_description}:${s.origin_stop}:${s.destination_stop}`
                    ).join('|')

                    // Group paths by their signature
                    const groups = new Map()
                    for (const { path } of directionPaths) {
                      const segments = collapseSteps(path.steps)
                      const sig = getSignature(segments)
                      if (!groups.has(sig)) {
                        groups.set(sig, { segments, steps: path.steps, times: [] })
                      }
                      const firstStep = path.steps[0]
                      const lastStep = path.steps[path.steps.length - 1]
                      groups.get(sig).times.push({
                        start: firstStep.origin_time,
                        end: lastStep.destination_time
                      })
                    }

                    return [...groups.values()].map((group, groupIndex) => {
                      // Build the list of stop IDs for this route from uncollapsed steps
                      const routeStops = []
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
