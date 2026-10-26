import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject } from "graphics-debug"
import type { CapacityMeshNode } from "lib/types"
import { createRectFromCapacityNode } from "lib/utils/createRectFromCapacityNode"
import { prepareTopologyMergingInput } from "./topology-merging-input"
import {
  type TopologyMergingOutputProvenance,
  createTopologyMergingOutputNodes,
  validateTopologyMergingOutput,
} from "./topology-merging-output"
import {
  compactTopologyMergingRegions,
  doesBoundsContainPoint,
  getCanonicalCoordinates,
  getLayerTopologiesForCoveredNodes,
  restoreAuthoritativeTargetRegions,
} from "./topology-merging-regions"
import { TOPOLOGY_MERGING_EPSILON } from "./topology-merging-types"
import type {
  PreparedTopologyMergingNode,
  TopologyMergingRegion,
  TopologyMergingSolverParams,
} from "./topology-merging-types"

export type {
  TopologyMergingNodeGroup,
  TopologyMergingSolverParams,
} from "./topology-merging-types"

type TopologyMergingXSlab = {
  minX: number
  maxX: number
  nodes: PreparedTopologyMergingNode[]
}

function hasInteriorOverlap(
  a: PreparedTopologyMergingNode,
  b: PreparedTopologyMergingNode,
): boolean {
  return (
    a.bounds.minX < b.bounds.maxX - TOPOLOGY_MERGING_EPSILON &&
    a.bounds.maxX > b.bounds.minX + TOPOLOGY_MERGING_EPSILON &&
    a.bounds.minY < b.bounds.maxY - TOPOLOGY_MERGING_EPSILON &&
    a.bounds.maxY > b.bounds.minY + TOPOLOGY_MERGING_EPSILON
  )
}

function sharesAvailableLayer(
  a: PreparedTopologyMergingNode,
  b: PreparedTopologyMergingNode,
): boolean {
  for (const z of a.node.availableZ) {
    if (b.node.availableZ.includes(z)) return true
  }
  return false
}

function buildFullXSlabs(
  preparedNodes: PreparedTopologyMergingNode[],
): TopologyMergingXSlab[] {
  const xCoordinates = getCanonicalCoordinates(
    preparedNodes.flatMap(({ bounds }) => [bounds.minX, bounds.maxX]),
  )
  const xSlabs: TopologyMergingXSlab[] = []
  for (let xIndex = 0; xIndex < xCoordinates.length - 1; xIndex++) {
    const minX = xCoordinates[xIndex]!
    const maxX = xCoordinates[xIndex + 1]!
    if (maxX - minX <= TOPOLOGY_MERGING_EPSILON) continue
    const x = (minX + maxX) / 2
    const nodesInXSlab = preparedNodes.filter(
      ({ bounds }) =>
        x >= bounds.minX - TOPOLOGY_MERGING_EPSILON &&
        x <= bounds.maxX + TOPOLOGY_MERGING_EPSILON,
    )
    if (nodesInXSlab.length > 0) {
      xSlabs.push({ minX, maxX, nodes: nodesInXSlab })
    }
  }
  return xSlabs
}

function buildInteractiveClustersAndPassthroughRegions(
  preparedNodes: PreparedTopologyMergingNode[],
): {
  passthroughRegions: TopologyMergingRegion[]
  xSlabs: TopologyMergingXSlab[]
} {
  const sortedByMinX = [...preparedNodes].sort(
    (a, b) => a.bounds.minX - b.bounds.minX,
  )
  const interactiveKeys = new Set<string>()
  const overlapAdjacency = new Map<string, PreparedTopologyMergingNode[]>()

  for (let i = 0; i < sortedByMinX.length; i++) {
    const nodeA = sortedByMinX[i]!
    for (let j = i + 1; j < sortedByMinX.length; j++) {
      const nodeB = sortedByMinX[j]!
      if (nodeB.bounds.minX >= nodeA.bounds.maxX - TOPOLOGY_MERGING_EPSILON) {
        break
      }
      if (nodeA.groupIndex === nodeB.groupIndex) continue
      if (!hasInteriorOverlap(nodeA, nodeB)) continue
      if (!sharesAvailableLayer(nodeA, nodeB)) continue

      interactiveKeys.add(nodeA.sourceKey)
      interactiveKeys.add(nodeB.sourceKey)

      let listA = overlapAdjacency.get(nodeA.sourceKey)
      if (!listA) {
        listA = []
        overlapAdjacency.set(nodeA.sourceKey, listA)
      }
      listA.push(nodeB)

      let listB = overlapAdjacency.get(nodeB.sourceKey)
      if (!listB) {
        listB = []
        overlapAdjacency.set(nodeB.sourceKey, listB)
      }
      listB.push(nodeA)
    }
  }

  const passthroughRegions: TopologyMergingRegion[] = []
  const interactiveNodes: PreparedTopologyMergingNode[] = []

  for (const preparedNode of preparedNodes) {
    if (interactiveKeys.has(preparedNode.sourceKey)) {
      interactiveNodes.push(preparedNode)
      continue
    }
    const mode =
      preparedNode.node._containsObstacle && preparedNode.node._containsTarget
        ? ("target-passthrough" as const)
        : ("passthrough" as const)
    passthroughRegions.push({
      bounds: { ...preparedNode.bounds },
      availableZ: [...preparedNode.node.availableZ],
      sourceKeys: [preparedNode.sourceKey],
      topologyMode: mode,
      topologySignature: JSON.stringify({
        mode,
        sourceKeys: [preparedNode.sourceKey],
      }),
    })
  }

  const visitedKeys = new Set<string>()
  const xSlabs: TopologyMergingXSlab[] = []

  for (const seedNode of interactiveNodes) {
    if (visitedKeys.has(seedNode.sourceKey)) continue
    const clusterNodes: PreparedTopologyMergingNode[] = []
    const queue: PreparedTopologyMergingNode[] = [seedNode]
    visitedKeys.add(seedNode.sourceKey)

    while (queue.length > 0) {
      const current = queue.pop()!
      clusterNodes.push(current)
      for (const neighbor of overlapAdjacency.get(current.sourceKey) ?? []) {
        if (!visitedKeys.has(neighbor.sourceKey)) {
          visitedKeys.add(neighbor.sourceKey)
          queue.push(neighbor)
        }
      }
    }

    const xCoordinates = getCanonicalCoordinates(
      clusterNodes.flatMap(({ bounds }) => [bounds.minX, bounds.maxX]),
    )
    for (let xIndex = 0; xIndex < xCoordinates.length - 1; xIndex++) {
      const minX = xCoordinates[xIndex]!
      const maxX = xCoordinates[xIndex + 1]!
      if (maxX - minX <= TOPOLOGY_MERGING_EPSILON) continue
      const x = (minX + maxX) / 2
      const nodesInXSlab = clusterNodes.filter(
        ({ bounds }) =>
          x >= bounds.minX - TOPOLOGY_MERGING_EPSILON &&
          x <= bounds.maxX + TOPOLOGY_MERGING_EPSILON,
      )
      if (nodesInXSlab.length > 0) {
        xSlabs.push({ minX, maxX, nodes: nodesInXSlab })
      }
    }
  }

  return { passthroughRegions, xSlabs }
}

export class TopologyMergingSolver extends BaseSolver {
  private readonly preparedNodes: PreparedTopologyMergingNode[]
  private readonly preparedNodeBySourceKey: Map<
    string,
    PreparedTopologyMergingNode
  >
  private readonly outputProvenance: TopologyMergingOutputProvenance = {
    groupIndexesByNodeId: new Map<string, number[]>(),
    sourceKeysByNodeId: new Map<string, string[]>(),
  }
  private readonly passthroughRegions: TopologyMergingRegion[]
  private readonly xSlabs: TopologyMergingXSlab[]
  private readonly atomicRegions: TopologyMergingRegion[] = []
  private outputNodes: CapacityMeshNode[] = []
  private currentXIndex = 0

  constructor(public readonly inputProblem: TopologyMergingSolverParams) {
    super()
    this.MAX_ITERATIONS = 100_000

    const { preparedNodes, preparedNodeBySourceKey } =
      prepareTopologyMergingInput(inputProblem)
    this.preparedNodes = preparedNodes
    this.preparedNodeBySourceKey = preparedNodeBySourceKey
    const { passthroughRegions, xSlabs } =
      inputProblem.nodeGroups.length <= 1
        ? { passthroughRegions: [], xSlabs: [] }
        : this.preparedNodes.length <= 500
          ? {
              passthroughRegions: [],
              xSlabs: buildFullXSlabs(this.preparedNodes),
            }
          : buildInteractiveClustersAndPassthroughRegions(this.preparedNodes)
    this.passthroughRegions = passthroughRegions
    this.xSlabs = xSlabs
    this.stats = {
      inputNodeCount: this.preparedNodes.length,
      xSlabCount: this.xSlabs.length,
      processedXSlabCount: 0,
      atomicRegionCount: 0,
      outputNodeCount: 0,
    }
  }

  override getConstructorParams(): [TopologyMergingSolverParams] {
    return [this.inputProblem]
  }

  override _step(): void {
    if (this.inputProblem.nodeGroups.length === 1) {
      this.completePassthroughTopology()
      return
    }

    if (this.currentXIndex < this.xSlabs.length) {
      this.processCurrentXSlab()
      this.currentXIndex += 1
      this.stats.processedXSlabCount = this.currentXIndex
      this.stats.atomicRegionCount = this.atomicRegions.length
      return
    }

    const topologyRegions = restoreAuthoritativeTargetRegions({
      regions: this.atomicRegions,
      preparedNodeBySourceKey: this.preparedNodeBySourceKey,
    })
    const compactedRegions = [
      ...this.passthroughRegions,
      ...compactTopologyMergingRegions(topologyRegions),
    ]
    this.outputNodes = createTopologyMergingOutputNodes({
      regions: compactedRegions,
      preparedNodeBySourceKey: this.preparedNodeBySourceKey,
      nodeGroups: this.inputProblem.nodeGroups,
      provenance: this.outputProvenance,
    })
    validateTopologyMergingOutput({
      nodes: this.outputNodes,
      preparedNodeBySourceKey: this.preparedNodeBySourceKey,
      provenance: this.outputProvenance,
    })
    this.stats.outputNodeCount = this.outputNodes.length
    this.stats.compactedRegionCount = compactedRegions.length
    this.solved = true
  }

  override getOutput(): CapacityMeshNode[] {
    if (!this.solved) {
      throw new Error(
        "TopologyMergingSolver: getOutput() called before the solver completed",
      )
    }

    return this.outputNodes
  }

  computeProgress(): number {
    const slabCount = Math.max(1, this.xSlabs.length)
    if (this.solved) return 1
    return Math.min(0.99, this.currentXIndex / slabCount)
  }

  override visualize(): GraphicsObject {
    const nodes = this.solved
      ? this.outputNodes
      : createTopologyMergingOutputNodes({
          regions: [...this.passthroughRegions, ...this.atomicRegions],
          preparedNodeBySourceKey: this.preparedNodeBySourceKey,
          nodeGroups: this.inputProblem.nodeGroups,
          provenance: this.outputProvenance,
          preserveSourceIds: false,
        })

    return {
      title: `Topology Merging: ${nodes.length} refined regions`,
      coordinateSystem: "cartesian",
      rects: nodes.map((node) => {
        const rect = createRectFromCapacityNode(node, {
          rectMargin: 0.01,
          zOffset: 0.02,
        })
        return {
          ...rect,
          label: `${node.capacityMeshNodeId}\navailableZ: ${node.availableZ.join(",")}\ncomponent: ${node._isComponentTopologyNode ? "yes" : "no"}`,
        }
      }),
      lines: [],
      points: [],
      circles: [],
      texts: [],
    }
  }

  private completePassthroughTopology(): void {
    const passthroughGroup = this.inputProblem.nodeGroups[0]!
    this.outputNodes = passthroughGroup.nodes
    for (const node of this.outputNodes) {
      this.outputProvenance.groupIndexesByNodeId.set(
        node.capacityMeshNodeId,
        [0],
      )
      this.outputProvenance.sourceKeysByNodeId.set(node.capacityMeshNodeId, [
        `${passthroughGroup.groupId}:${node.capacityMeshNodeId}`,
      ])
    }
    this.stats.processedXSlabCount = this.stats.xSlabCount
    this.stats.atomicRegionCount = 0
    this.stats.compactedRegionCount = 0
    this.stats.outputNodeCount = this.outputNodes.length
    this.stats.passthroughNodeCount = this.outputNodes.length
    this.solved = true
  }

  private processCurrentXSlab(): void {
    const slab = this.xSlabs[this.currentXIndex]!
    const { minX, maxX, nodes: nodesInXSlab } = slab
    if (maxX - minX <= TOPOLOGY_MERGING_EPSILON) return

    const x = (minX + maxX) / 2
    const yCoordinates = getCanonicalCoordinates(
      nodesInXSlab.flatMap(({ bounds }) => [bounds.minY, bounds.maxY]),
    )

    for (let yIndex = 0; yIndex < yCoordinates.length - 1; yIndex++) {
      const minY = yCoordinates[yIndex]!
      const maxY = yCoordinates[yIndex + 1]!
      if (maxY - minY <= TOPOLOGY_MERGING_EPSILON) continue

      const point = { x, y: (minY + maxY) / 2 }
      const coveringNodes = nodesInXSlab.filter(({ bounds }) =>
        doesBoundsContainPoint(bounds, point),
      )
      if (coveringNodes.length === 0) continue

      const layerTopologies = getLayerTopologiesForCoveredNodes({
        coveringNodes,
        nodeGroups: this.inputProblem.nodeGroups,
        layerCount: this.inputProblem.layerCount,
      })
      for (const layerTopology of layerTopologies) {
        this.atomicRegions.push({
          bounds: { minX, maxX, minY, maxY },
          availableZ: layerTopology.availableZ,
          sourceKeys: layerTopology.sourceKeys,
          topologyMode: layerTopology.topologyMode,
          topologySignature: layerTopology.topologySignature,
        })
      }
    }
  }
}
