import { expect, test } from "bun:test"
import type { SharedEdgeSegment } from "../../lib/solvers/AvailableSegmentPointSolver/AvailableSegmentPointSolver"
import { MultiTargetNecessaryCrampedPortPointSolver } from "../../lib/solvers/NecessaryCrampedPortPointSolver/MultiTargetNecessaryCrampedPortPointSolver"
import type { CapacityMeshNode, SimpleRouteJson } from "../../lib/types"

const createNode = (
  id: string,
  x: number,
  containsObstacle = false,
): CapacityMeshNode => ({
  capacityMeshNodeId: id,
  center: { x, y: 0 },
  width: 1,
  height: 1,
  layer: "top",
  availableZ: [0],
  _containsObstacle: containsObstacle,
})

const createSegment = (
  edgeId: string,
  nodeA: string,
  nodeB: string,
  x: number,
  cramped: boolean,
): SharedEdgeSegment => ({
  edgeId,
  nodeIds: [nodeA, nodeB],
  start: { x, y: -0.5 },
  end: { x, y: 0.5 },
  length: 1,
  availableZ: [0],
  isCramped: cramped,
  portPoints: [
    {
      segmentPortPointId: `${edgeId}_pp0`,
      x,
      y: 0,
      availableZ: [0],
      connectionName: null,
      distToCenterMostPortOnZ: 0,
      cramped,
      nodeIds: [nodeA, nodeB],
    },
  ],
})

test("MultiTargetNecessaryCrampedPortPointSolver retains cramped bridge port when target sits inside a multi-hop single-layer pocket", () => {
  const capacityMeshNodes: CapacityMeshNode[] = [
    createNode("target-start", 0, true),
    createNode("free-1", 1, false),
    createNode("free-2", 2, false),
    createNode("free-3", 3, false),
    createNode("free-4", 4, false),
    createNode("target-end", 5, true),
  ]

  const sharedEdgeSegments: SharedEdgeSegment[] = [
    createSegment("edge-0-1", "target-start", "free-1", 0.5, false),
    createSegment("edge-1-2", "free-1", "free-2", 1.5, false),
    createSegment("edge-2-3", "free-2", "free-3", 2.5, false),
    createSegment("edge-3-4-cramped", "free-3", "free-4", 3.5, true),
    createSegment("edge-4-5", "free-4", "target-end", 4.5, false),
  ]

  const simpleRouteJson: SimpleRouteJson = {
    layerCount: 2,
    minTraceWidth: 0.15,
    bounds: { minX: -1, maxX: 6, minY: -2, maxY: 2 },
    obstacles: [],
    connections: [
      {
        name: "conn-1",
        pointsToConnect: [
          { x: 0, y: 0, layer: "top" },
          { x: 5, y: 0, layer: "top" },
        ],
      },
    ],
  }

  const solver = new MultiTargetNecessaryCrampedPortPointSolver({
    capacityMeshNodes,
    sharedEdgeSegments,
    simpleRouteJson,
  })
  solver.solve()

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)

  const output = solver.getOutput()
  const bridgeSegment = output.find(
    (segment) => segment.edgeId === "edge-3-4-cramped",
  )
  expect(bridgeSegment?.portPoints).toHaveLength(1)
  expect(bridgeSegment?.portPoints[0]?.tinyHypergraphPortPenalty).toBe(1000)
})
