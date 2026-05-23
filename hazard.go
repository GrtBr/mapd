package main

import (
	"pfeifer.dev/mapd/maps"
)

func checkWayForHazardChange(state *State, parent *Upcoming[string], way maps.NextWayResult) (valid bool, val string) {
	nextHazard := way.Way.Hazard()

	// Node-level hazard at the junction point takes priority over way-level hazard
	if nodeHazard := way.Way.NodeHazardAtJunction(way.IsForward); nodeHazard != "" {
		nextHazard = nodeHazard
	}

	// Fallback: a hazard on the EXIT node of the way the driver is currently on.
	// Synthetic T-Junction hazards are deliberately tagged only on the stem way
	// (the road that must yield) — never on the through-road that the next way
	// represents — so the NextWays checks above miss them. The stem road is the
	// current way as the driver approaches; its exit node carries the hazard.
	// distToEnd(CurrentWay) — Upcoming.Update()'s base distance for the first
	// NextWay — is exactly the distance to that exit node.
	if nextHazard == "" {
		if exitHazard := state.CurrentWay.Way.NodeHazardAtJunction(!state.CurrentWay.OnWay.IsForward); exitHazard != "" {
			nextHazard = exitHazard
		}
	}

	if nextHazard != state.CurrentWay.Way.Hazard() && nextHazard != "" {
		return true, nextHazard
	}

	return false, parent.DefaultValue
}
