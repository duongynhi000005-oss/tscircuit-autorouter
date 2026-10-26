import { describe, expect, test } from "bun:test"
import { AutoroutingPipelineSolver9_PreloadedTraceGraph } from "lib/autorouter-pipelines/AutoroutingPipeline9_PreloadedTraceGraph/AutoroutingPipelineSolver9_PreloadedTraceGraph"
import { NodeDimensionSubdivisionSolver } from "lib/solvers/NodeDimensionSubdivisionSolver/NodeDimensionSubdivisionSolver"
import { SelectiveReripTinyHyperGraphSolverWithStableInitialAssignments } from "lib/solvers/PortPointPathingSolver/tinyhypergraph/SelectiveReripTinyHyperGraphSolverWithStableInitialAssignments"
import { TopologyMergingSolver } from "lib/solvers/TopologyMergingSolver/TopologyMergingSolver"
import type { CapacityMeshNode, SimpleRouteJson } from "lib/types"

describe("Pipeline9 topology merging, sliver subdivision, single-layer rerip, and diffpair validation", () => {
  test("TopologyMergingSolver passes through isolated groups and merges only overlapping groups", () => {
    const nodeGroups: Array<{
      groupId: string
      nodes: CapacityMeshNode[]
      isComponent: boolean
    }> = []

    // 400 non-overlapping component groups spread along X/Y
    for (let i = 0; i < 400; i++) {
      const gx = (i % 20) * 10
      const gy = Math.floor(i / 20) * 10
      nodeGroups.push({
        groupId: `isolated_${i}`,
        isComponent: true,
        nodes: [
          {
            capacityMeshNodeId: `iso_${i}_a`,
            center: { x: gx, y: gy },
            width: 2,
            height: 2,
            layer: "top",
            availableZ: [0, 1],
          },
          {
            capacityMeshNodeId: `iso_${i}_b`,
            center: { x: gx + 2, y: gy },
            width: 2,
            height: 2,
            layer: "top",
            availableZ: [0, 1],
          },
        ],
      })
    }

    // Two overlapping groups that require intersection merging
    nodeGroups.push({
      groupId: "overlap_a",
      isComponent: false,
      nodes: [
        {
          capacityMeshNodeId: "ov_a",
          center: { x: 500, y: 500 },
          width: 4,
          height: 4,
          layer: "top",
          availableZ: [0, 1],
        },
      ],
    })
    nodeGroups.push({
      groupId: "overlap_b",
      isComponent: true,
      nodes: [
        {
          capacityMeshNodeId: "ov_b",
          center: { x: 501, y: 501 },
          width: 4,
          height: 4,
          layer: "top",
          availableZ: [0],
          _containsObstacle: true,
        },
      ],
    })

    const startMs = performance.now()
    const solver = new TopologyMergingSolver({
      nodeGroups,
      layerCount: 2,
    })
    solver.solve()
    const elapsedMs = performance.now() - startMs

    expect(solver.solved).toBe(true)
    expect(solver.failed).toBe(false)
    expect(elapsedMs).toBeLessThan(2000)
    expect(solver.getOutput().length).toBeGreaterThan(800)
  })

  test("NodeDimensionSubdivisionSolver removes long coordinate slivers thinner than 0.002mm even when area exceeds minNodeArea", () => {
    const sliverNode: CapacityMeshNode = {
      capacityMeshNodeId: "long-sliver",
      center: { x: 0, y: 0 },
      width: 0.0007,
      height: 20, // area = 0.014 >= default minNodeArea (0.01)
      layer: "top",
      availableZ: [0, 1, 2, 3],
    }
    const normalNode: CapacityMeshNode = {
      capacityMeshNodeId: "normal-node",
      center: { x: 5, y: 0 },
      width: 1,
      height: 1,
      layer: "top",
      availableZ: [0, 1, 2, 3],
    }

    const solver = new NodeDimensionSubdivisionSolver(
      [sliverNode, normalNode],
      100,
      2,
    )
    solver.solve()

    expect(solver.solved).toBe(true)
    expect(solver.stats.removedSmallNodeCount).toBe(1)
    expect(solver.outputNodes.map((n) => n.capacityMeshNodeId)).toEqual([
      "normal-node",
    ])
  })

  test("SelectiveReripTinyHyperGraphSolverWithStableInitialAssignments increments single-layer conflict congestion when existingRegionCost is 0", () => {
    const topology = {
      portCount: 6,
      regionCount: 3,
      regionIncidentPorts: [
        [0, 1, 2, 3],
        [0, 1],
        [2, 3],
      ],
      incidentPortRegion: [[0, 1], [0, 1], [0, 2], [0, 2], [1], [2]],
      regionWidth: new Float64Array([1, 1, 1]),
      regionHeight: new Float64Array([1, 1, 1]),
      regionCenterX: new Float64Array([0, -1, 1]),
      regionCenterY: new Float64Array([0, 0, 0]),
      regionAvailableZMask: new Int32Array([1, 1, 1]),
      portCenterX: new Float64Array([-0.5, 0, 0.5, 0, -1.5, 1.5]),
      portCenterY: new Float64Array([0, -0.5, 0, 0.5, 0, 0]),
      portX: new Float64Array([-0.5, 0, 0.5, 0, -1.5, 1.5]),
      portY: new Float64Array([0, -0.5, 0, 0.5, 0, 0]),
      portZ: new Int32Array([0, 0, 0, 0, 0, 0]),
      portAngleForRegion1: new Int32Array([0, 900, 1800, 2700, 0, 0]),
      portAngleForRegion2: new Int32Array([0, 900, 1800, 2700, 0, 0]),
    }
    const problem = {
      routeCount: 2,
      portNetId: new Int32Array([0, 1, 0, 1, 0, 1]),
      routeNet: new Int32Array([0, 1]),
      routeStartPort: new Int32Array([0, 1]),
      routeEndPort: new Int32Array([2, 3]),
      regionNetId: new Int32Array([-1, -1, -1]),
    }

    const solver =
      new SelectiveReripTinyHyperGraphSolverWithStableInitialAssignments(
        topology as any,
        problem as any,
      )
    // Route 0 occupies ports 0 -> 2 through single-layer region 0 (zMask = 1),
    // leaving existingRegionCost[0] === 0 while blocking crossing route 1 (ports 1 -> 3).
    solver.state.portAssignment[0] = 0
    solver.state.portAssignment[2] = 0
    solver.state.regionSegments[0] = [[0, 0, 2]]
    ;(solver as any).appendSegmentToRegionCache(0, 0, 2)
    solver.state.unroutedRoutes = [1]
    solver.state.currentRouteId = 1
    solver.state.currentRouteNetId = 1
    solver.state.goalPortId = 3

    const beforeCongestion = solver.state.regionCongestionCost[0]!
    const result = (
      solver as any
    ).findRelaxedBlockerPathPreferringPreservedRoutes(new Set<number>())

    expect(result.found).toBe(true)
    expect(solver.state.regionCongestionCost[0]!).toBeGreaterThan(
      beforeCongestion,
    )
  })

  test("Pipeline9 fails early at netToPointPairsSolver when a differential pair connection splits into multiple point-pair segments", () => {
    const srj: SimpleRouteJson = {
      layerCount: 2,
      minTraceWidth: 0.15,
      bounds: { minX: -10, maxX: 10, minY: -10, maxY: 10 },
      obstacles: [],
      connections: [
        {
          name: "diff_pos",
          pointsToConnect: [
            { x: -5, y: -2, layer: "top" },
            { x: 0, y: -2, layer: "top" },
            { x: 5, y: -2, layer: "top" },
            { x: 8, y: -2, layer: "top" },
          ],
        },
        {
          name: "diff_neg",
          pointsToConnect: [
            { x: -5, y: 2, layer: "top" },
            { x: 0, y: 2, layer: "top" },
            { x: 5, y: 2, layer: "top" },
            { x: 8, y: 2, layer: "top" },
          ],
        },
      ],
      differentialPairs: [
        {
          connectionNames: ["diff_pos", "diff_neg"],
          lengthTolerance: 0.1,
          traceGap: 0.15,
        },
      ],
    }

    const solver = new AutoroutingPipelineSolver9_PreloadedTraceGraph(srj)
    solver.solve()

    expect(solver.failed).toBe(true)
    expect(solver.solved).toBe(false)
    expect(solver.error).toBe(
      'Pipeline9: differential pair connection "diff_pos" must resolve to exactly one final point-pair connection, got 3',
    )
    // Must halt immediately at netToPointPairsSolver before topologyPlanningSolver / topologyMergingSolver run
    expect(solver.topologyPlanningSolver).toBeUndefined()
    expect(solver.topologyMergingSolver).toBeUndefined()
  })
})
