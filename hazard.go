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

	if nextHazard != state.CurrentWay.Way.Hazard() && nextHazard != "" {
		return true, nextHazard
	}

	return false, parent.DefaultValue
}
