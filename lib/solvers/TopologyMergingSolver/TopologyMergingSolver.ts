import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject } from "graphics-debug"
import type { CapacityMeshNode } from "lib/types"
import { createRectFromCapacityNode } from "lib/utils/createRectFromCapacityNode"
import { prepareTopologyMergingInput } from "./topology-merging-input"
import {
  createTopologyMergingOutputNodes,
  type TopologyMergingOutputProvenance,
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

type TopologyMergingSlabTask = {
  nodes: PreparedTopologyMergingNode[]
  xCoordinates: number[]
  currentXIndex: number
  atomicRegions: TopologyMergingRegion[]
}

function isTargetObstacleNode(
  preparedNode: PreparedTopologyMergingNode,
): boolean {
  return Boolean(
    preparedNode.node._containsObstacle && preparedNode.node._containsTarget,
  )
}

function doNodesShareLayer(
  a: PreparedTopologyMergingNode,
  b: PreparedTopologyMergingNode,
): boolean {
  const layersA = a.node.availableZ
  const layersB = b.node.availableZ
  for (let i = 0; i < layersA.length; i++) {
    if (layersB.includes(layersA[i]!)) return true
  }
  return false
}

function doNodesInteractInTopologyMerging(
  a: PreparedTopologyMergingNode,
  b: PreparedTopologyMergingNode,
): boolean {
  if (a.groupIndex === b.groupIndex) return false
  if (!doNodesShareLayer(a, b)) return false
  return (
    Math.min(a.bounds.maxX, b.bounds.maxX) -
      Math.max(a.bounds.minX, b.bounds.minX) >
      TOPOLOGY_MERGING_EPSILON &&
    Math.min(a.bounds.maxY, b.bounds.maxY) -
      Math.max(a.bounds.minY, b.bounds.minY) >
      TOPOLOGY_MERGING_EPSILON
  )
}

function createPassthroughRegionForNode(
  preparedNode: PreparedTopologyMergingNode,
): TopologyMergingRegion {
  const topologyMode = isTargetObstacleNode(preparedNode)
    ? ("target-passthrough" as const)
    : ("passthrough" as const)
  return {
    bounds: { ...preparedNode.bounds },
    availableZ: [...preparedNode.node.availableZ],
    sourceKeys: [preparedNode.sourceKey],
    topologyMode,
    topologySignature: JSON.stringify({
      mode: topologyMode,
      sourceKeys: [preparedNode.sourceKey],
    }),
  }
}

function partitionPreparedNodesIntoTasks(
  preparedNodes: PreparedTopologyMergingNode[],
): {
  passthroughNodes: PreparedTopologyMergingNode[]
  interactiveClusters: PreparedTopologyMergingNode[][]
} {
  const count = preparedNodes.length
  if (count === 0) {
    return { passthroughNodes: [], interactiveClusters: [] }
  }

  const parent = new Int32Array(count)
  const interacts = new Uint8Array(count)
  for (let i = 0; i < count; i++) parent[i] = i

  const find = (x: number): number => {
    let root = x
    while (parent[root] !== root) root = parent[root]!
    let curr = x
    while (curr !== root) {
      const next = parent[curr]!
      parent[curr] = root
      curr = next
    }
    return root
  }

  const union = (a: number, b: number): void => {
    const rootA = find(a)
    const rootB = find(b)
    if (rootA !== rootB) parent[rootB] = rootA
  }

  const sortedIndices = Array.from({ length: count }, (_, i) => i).sort(
    (a, b) => preparedNodes[a]!.bounds.minX - preparedNodes[b]!.bounds.minX,
  )

  for (let i = 0; i < count; i++) {
    const idxA = sortedIndices[i]!
    const nodeA = preparedNodes[idxA]!
    const boundsA = nodeA.bounds
    for (let j = i + 1; j < count; j++) {
      const idxB = sortedIndices[j]!
      const nodeB = preparedNodes[idxB]!
      const boundsB = nodeB.bounds
      if (boundsB.minX >= boundsA.maxX - TOPOLOGY_MERGING_EPSILON) {
        break
      }
      if (doNodesInteractInTopologyMerging(nodeA, nodeB)) {
        interacts[idxA] = 1
        interacts[idxB] = 1
        union(idxA, idxB)
      }
    }
  }

  const passthroughNodes: PreparedTopologyMergingNode[] = []
  const clustersByRoot = new Map<number, PreparedTopologyMergingNode[]>()
  for (let i = 0; i < count; i++) {
    if (!interacts[i]) {
      passthroughNodes.push(preparedNodes[i]!)
      continue
    }
    const root = find(i)
    let cluster = clustersByRoot.get(root)
    if (!cluster) {
      cluster = []
      clustersByRoot.set(root, cluster)
    }
    cluster.push(preparedNodes[i]!)
  }

  return {
    passthroughNodes,
    interactiveClusters: [...clustersByRoot.values()],
  }
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
  private readonly passthroughRegions: TopologyMergingRegion[] = []
  private readonly compactedClusterRegions: TopologyMergingRegion[] = []
  private readonly slabTasks: TopologyMergingSlabTask[] = []
  private currentTaskIndex = 0
  private totalXSlabs = 0
  private processedXSlabs = 0
  private totalAtomicRegions = 0
  private outputNodes: CapacityMeshNode[] = []

  constructor(public readonly inputProblem: TopologyMergingSolverParams) {
    super()
    this.MAX_ITERATIONS = 100_000

    const { preparedNodes, preparedNodeBySourceKey } =
      prepareTopologyMergingInput(inputProblem)
    this.preparedNodes = preparedNodes
    this.preparedNodeBySourceKey = preparedNodeBySourceKey

    if (this.inputProblem.nodeGroups.length > 1) {
      const { passthroughNodes, interactiveClusters } =
        partitionPreparedNodesIntoTasks(this.preparedNodes)
      for (const node of passthroughNodes) {
        this.passthroughRegions.push(createPassthroughRegionForNode(node))
      }
      for (const cluster of interactiveClusters) {
        const xCoordinates = getCanonicalCoordinates(
          cluster.flatMap(({ bounds }) => [bounds.minX, bounds.maxX]),
        )
        const slabCount = Math.max(0, xCoordinates.length - 1)
        if (slabCount > 0) {
          this.slabTasks.push({
            nodes: cluster,
            xCoordinates,
            currentXIndex: 0,
            atomicRegions: [],
          })
          this.totalXSlabs += slabCount
        }
      }
      this.totalAtomicRegions = this.passthroughRegions.length
    }

    this.stats = {
      inputNodeCount: this.preparedNodes.length,
      xSlabCount: this.totalXSlabs,
      processedXSlabCount: 0,
      atomicRegionCount: this.totalAtomicRegions,
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

    while (this.currentTaskIndex < this.slabTasks.length) {
      const task = this.slabTasks[this.currentTaskIndex]!
      if (task.currentXIndex < task.xCoordinates.length - 1) {
        this.processTaskXSlab(task)
        task.currentXIndex += 1
        this.processedXSlabs += 1
        this.stats.processedXSlabCount = this.processedXSlabs
        this.stats.atomicRegionCount = this.totalAtomicRegions
        if (task.currentXIndex >= task.xCoordinates.length - 1) {
          this.finalizeSlabTask(task)
          this.currentTaskIndex += 1
        }
        return
      }
      this.finalizeSlabTask(task)
      this.currentTaskIndex += 1
    }

    const allCompactedRegions = [
      ...this.passthroughRegions,
      ...this.compactedClusterRegions,
    ]
    this.outputNodes = createTopologyMergingOutputNodes({
      regions: allCompactedRegions,
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
    this.stats.compactedRegionCount = allCompactedRegions.length
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
    const slabCount = Math.max(1, this.totalXSlabs)
    if (this.solved) return 1
    return Math.min(0.99, this.processedXSlabs / slabCount)
  }

  override visualize(): GraphicsObject {
    const activeAtomicRegions = this.slabTasks
      .slice(this.currentTaskIndex)
      .flatMap((task) => task.atomicRegions)
    const nodes = this.solved
      ? this.outputNodes
      : createTopologyMergingOutputNodes({
          regions: [
            ...this.passthroughRegions,
            ...this.compactedClusterRegions,
            ...activeAtomicRegions,
          ],
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

  private finalizeSlabTask(task: TopologyMergingSlabTask): void {
    if (task.atomicRegions.length === 0) return
    const topologyRegions = restoreAuthoritativeTargetRegions({
      regions: task.atomicRegions,
      preparedNodeBySourceKey: this.preparedNodeBySourceKey,
    })
    const compactedRegions = compactTopologyMergingRegions(topologyRegions)
    this.compactedClusterRegions.push(...compactedRegions)
    task.atomicRegions.length = 0
  }

  private processTaskXSlab(task: TopologyMergingSlabTask): void {
    const minX = task.xCoordinates[task.currentXIndex]!
    const maxX = task.xCoordinates[task.currentXIndex + 1]!
    if (maxX - minX <= TOPOLOGY_MERGING_EPSILON) return

    const x = (minX + maxX) / 2
    const nodesInXSlab = task.nodes.filter(
      ({ bounds }) =>
        x >= bounds.minX - TOPOLOGY_MERGING_EPSILON &&
        x <= bounds.maxX + TOPOLOGY_MERGING_EPSILON,
    )
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
        task.atomicRegions.push({
          bounds: { minX, maxX, minY, maxY },
          availableZ: layerTopology.availableZ,
          sourceKeys: layerTopology.sourceKeys,
          topologyMode: layerTopology.topologyMode,
          topologySignature: layerTopology.topologySignature,
        })
        this.totalAtomicRegions += 1
      }
    }
  }
}
