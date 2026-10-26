import { expect, test } from "bun:test"
import { AutoroutingPipelineSolver9_PreloadedTraceGraph } from "../../lib/autorouter-pipelines/AutoroutingPipeline9_PreloadedTraceGraph/AutoroutingPipelineSolver9_PreloadedTraceGraph"
import type { SimpleRouteJson } from "../../lib/types"

test("Pipeline9 completes when a differential-pair connection is split into multiple MST point-pair segments", async () => {
  const fixtureUrl = new URL(
    "../fixtures/core-differential-pair-pad-clearance.json",
    import.meta.url,
  )
  const input: SimpleRouteJson = await Bun.file(fixtureUrl).json()
  const pairConnectionName = input.differentialPairs![0]!.connectionNames[0]!
  const targetConnection = input.connections.find(
    (connection) => connection.name === pairConnectionName,
  )!
  const lastPoint =
    targetConnection.pointsToConnect[
      targetConnection.pointsToConnect.length - 1
    ]!
  targetConnection.pointsToConnect.push({
    ...lastPoint,
    x: lastPoint.x + 0.8,
    pcb_port_id: `${lastPoint.pcb_port_id ?? "port"}_branch`,
  })

  const solver = new AutoroutingPipelineSolver9_PreloadedTraceGraph(input)
  solver.solve()

  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(solver.getOutputSimpleRouteJson().traces?.length).toBeGreaterThan(0)
}, 30_000)
