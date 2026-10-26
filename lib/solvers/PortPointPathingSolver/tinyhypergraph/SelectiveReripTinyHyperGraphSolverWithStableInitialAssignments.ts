import { SelectiveReripTinyHyperGraphSolver } from "tiny-hypergraph/lib/index"
import { applyInitialAssignments } from "tiny-hypergraph/lib/initialAssignments"

/**
 * Selective rerips may move a preloaded assignment when it is the blocker.
 * A global retry restores the serialized assignments instead of eagerly
 * discarding every preloaded route.
 */
export class SelectiveReripTinyHyperGraphSolverWithStableInitialAssignments extends SelectiveReripTinyHyperGraphSolver {
  private initialAssignmentRouteIds?: ReadonlySet<number>

  protected override getRouteIdsPreferredForPreservation(): ReadonlySet<number> {
    if (!this.initialAssignmentRouteIds) {
      this.initialAssignmentRouteIds = new Set(
        (this.problem.initialAssignments ?? []).map(
          (assignment) => assignment.routeId,
        ),
      )
    }
    return this.initialAssignmentRouteIds
  }

  override resetRoutingStateForRerip(): void {
    super.resetRoutingStateForRerip()
    if (!this.problem.initialAssignments?.length) return

    applyInitialAssignments({
      topology: this.topology,
      problem: this.problem,
      state: this.state,
      routeSuccessCountByRouteId: this.routeSuccessCountByRouteId,
      appendSegmentToRegionCache: (regionId, fromPortId, toPortId) =>
        this.appendSegmentToRegionCache(regionId, fromPortId, toPortId),
    })
  }

  protected override findRelaxedBlockerPathPreferringPreservedRoutes(
    forbiddenOwnerRouteIds?: ReadonlySet<number>,
  ) {
    const blocker = super.findRelaxedBlockerPathPreferringPreservedRoutes(
      forbiddenOwnerRouteIds,
    )
    if (blocker.found) {
      for (const hop of blocker.hops) {
        for (const resource of hop.data?.resources ?? []) {
          if (resource.kind === "same_layer_intersection") {
            const existingRegionCost =
              this.state.regionIntersectionCaches[resource.regionId]
                ?.existingRegionCost ?? 0
            if (existingRegionCost === 0) {
              this.state.regionCongestionCost[resource.regionId] +=
                this.RIP_CONGESTION_REGION_COST_FACTOR
            }
          } else if (resource.kind === "port") {
            for (const regionId of this.topology.incidentPortRegion[
              resource.portId
            ] ?? []) {
              const existingRegionCost =
                this.state.regionIntersectionCaches[regionId]
                  ?.existingRegionCost ?? 0
              if (
                this.problem.regionNetId[regionId] === -1 &&
                existingRegionCost === 0
              ) {
                this.state.regionCongestionCost[regionId] +=
                  this.RIP_CONGESTION_REGION_COST_FACTOR * 0.5
              }
            }
          }
        }
      }
    }
    return blocker
  }
}
