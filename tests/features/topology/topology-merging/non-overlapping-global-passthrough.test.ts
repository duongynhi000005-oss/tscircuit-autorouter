import { expect, test } from "bun:test"
import { TopologyMergingSolver } from "../../../../lib/solvers/TopologyMergingSolver/TopologyMergingSolver"
import { createTopologyMergingTestNode } from "../../../fixtures/topology-merging-test-utils"

test("topology merging passes through global nodes that do not overlap any component group", (): void => {
  const distantGlobalNodes = Array.from({ length: 40 }, (_, index) =>
    createTopologyMergingTestNode({
      id: `global-${index}`,
      bounds: {
        minX: 100 + index * 10,
        maxX: 105 + index * 10,
        minY: 100 + index * 10,
        maxY: 105 + index * 10,
      },
      availableZ: [0, 1],
    }),
  )
  const overlappingGlobalNode = createTopologyMergingTestNode({
    id: "global-overlap",
    bounds: { minX: 0, maxX: 10, minY: 0, maxY: 10 },
    availableZ: [0, 1],
  })
  const componentNode = createTopologyMergingTestNode({
    id: "component-0",
    bounds: { minX: 2, maxX: 6, minY: 2, maxY: 6 },
    availableZ: [0],
  })

  const solver = new TopologyMergingSolver({
    layerCount: 2,
    nodeGroups: [
      {
        groupId: "global",
        isComponent: false,
        nodes: [overlappingGlobalNode, ...distantGlobalNodes],
      },
      {
        groupId: "component-a",
        isComponent: true,
        nodes: [componentNode],
      },
    ],
  })

  solver.solve()

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(solver.stats.xSlabCount).toBeLessThanOrEqual(4)
  const outputIds = new Set(
    solver.getOutput().map((node) => node.capacityMeshNodeId),
  )
  for (const node of distantGlobalNodes) {
    expect(outputIds.has(node.capacityMeshNodeId)).toBe(true)
  }
})
