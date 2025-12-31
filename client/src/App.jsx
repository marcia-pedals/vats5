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
  const { stopsById, nodes, nodePositions, edges } = useMemo(() => {
    if (!visualization) {
      return { stopsById: new Map(), nodes: [], nodePositions: new Map(), edges: [] }
    }

    const stopsById = new Map(visualization.stops.map(([id, stop]) => [id, stop]))
    const stops = [...stopsById.entries()]

    if (stops.length === 0) {
      return { stopsById, nodes: [], nodePositions: new Map(), edges: [] }
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

    // Run force simulation
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

    const nodePositions = new Map(nodes.map(n => [n.id, {
      x: padding + ((n.x - simMinX) / simRangeX) * (svgWidth - 2 * padding),
      y: padding + ((n.y - simMinY) / simRangeY) * (svgHeight - 2 * padding),
    }]))

    // Build edges
    const edgeSet = new Set()
    for (const [originId, stepGroups] of visualization.adjacent) {
      const destinations = new Set()
      for (const group of stepGroups) {
        for (const step of group) {
          destinations.add(step.destination_stop)
        }
      }
      for (const destId of destinations) {
        if (stopsById.has(originId) && stopsById.has(destId)) {
          const key = originId < destId ? `${originId}-${destId}` : `${destId}-${originId}`
          edgeSet.add(key)
        }
      }
    }
    const edges = [...edgeSet].map(key => {
      const [from, to] = key.split('-').map(Number)
      return { from, to }
    })

    return { stopsById, nodes, nodePositions, edges }
  }, [visualization])

  if (error) {
    return <div>{error}</div>
  }

  if (!visualization || nodes.length === 0) {
    return <div>Loading...</div>
  }

  return (
    <div style={{ position: 'relative' }}>
      <h1>VATS5</h1>
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
              strokeWidth={isSelected ? 7 : 5}
              style={{ cursor: 'pointer' }}
              onMouseEnter={(e) => {
                setHoveredEdge({ from: fromStop, to: toStop })
                setMousePos({ x: e.clientX, y: e.clientY })
              }}
              onMouseMove={(e) => {
                setMousePos({ x: e.clientX, y: e.clientY })
              }}
              onMouseLeave={() => setHoveredEdge(null)}
              onClick={() => setSelectedEdge({ from: edge.from, to: edge.to, fromStop, toStop })}
            />
          )
        })}
        {nodes.map((node) => {
          const pos = nodePositions.get(node.id)
          return (
            <circle
              key={node.id}
              cx={pos.x}
              cy={pos.y}
              r={5}
              fill="blue"
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
      </svg>
      <div style={{
        width: '200px',
        padding: '10px',
        border: '1px solid #ccc',
        borderRadius: '4px',
        height: 'fit-content'
      }}>
        <h3 style={{ margin: '0 0 10px 0' }}>Selected Edge</h3>
        {selectedEdge ? (
          <div>{selectedEdge.fromStop.stop_name} ↔ {selectedEdge.toStop.stop_name}</div>
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
