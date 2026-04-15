package main

import (
	"fmt"
	"math"

	"github.com/pkg/errors"
	m "pfeifer.dev/mapd/math"
	ms "pfeifer.dev/mapd/settings"
)

// minNodeSpacing is the minimum distance between consecutive nodes passed to
// GetCurvatures. Nodes closer than this are skipped by SubsamplePositions.
// Matches mapd_test.py MIN_NODE_SPACING = 20.0 m.
const minNodeSpacing float32 = 20.0

// SubsamplePositions returns a copy of positions where consecutive entries are
// at least minSpacing metres apart. The first node is always kept. The last node
// is always appended so the route endpoint is included.
// This is the correct way to suppress 1/spacing² noise in Heron's formula:
// skip intermediate nodes until the cumulative distance from the last kept node
// reaches the threshold, rather than checking consecutive pairs.
func SubsamplePositions(positions []m.Position, minSpacing float32) []m.Position {
	if len(positions) < 2 {
		return positions
	}
	sampled := make([]m.Position, 0, len(positions))
	sampled = append(sampled, positions[0])
	for _, p := range positions[1:] {
		if sampled[len(sampled)-1].DistanceTo(p) >= minSpacing {
			sampled = append(sampled, p)
		}
	}
	last := positions[len(positions)-1]
	if !sampled[len(sampled)-1].Equals(last) {
		sampled = append(sampled, last)
	}
	return sampled
}

func GetStateCurvatures(state *State) ([]m.Curvature, error) {
	nodes := state.CurrentWay.Way.Nodes()
	num_points := len(nodes)
	all_nodes := [][]m.Position{nodes}
	all_nodes_direction := []bool{state.CurrentWay.OnWay.IsForward}
	lastWay := state.CurrentWay.Way
	for _, nextWay := range state.NextWays {
		nwNodes := nextWay.Way.Nodes()
		if len(nwNodes) > 0 {
			num_points += len(nwNodes) - 1
		}
		all_nodes = append(all_nodes, nwNodes)
		all_nodes_direction = append(all_nodes_direction, nextWay.IsForward)
		lastWay = nextWay.Way
	}
	_ = lastWay

	positions := make([]m.Position, num_points)

	all_nodes_idx := 0
	nodes_idx := 0
	for i := 0; i < num_points; i++ {
		var index int
		forward := all_nodes_direction[all_nodes_idx]
		if forward {
			index = nodes_idx
			if all_nodes_idx > 0 {
				index += 1
			}
		} else {
			index = len(all_nodes[all_nodes_idx]) - nodes_idx - 1
			if all_nodes_idx > 0 {
				index -= 1
			}
		}
		positions[i] = all_nodes[all_nodes_idx][index]

		nodes_idx += 1
		if nodes_idx == len(all_nodes[all_nodes_idx]) || (nodes_idx == len(all_nodes[all_nodes_idx])-1 && all_nodes_idx > 0) {
			all_nodes_idx += 1
			nodes_idx = 0
		}
	}

	// Resample to at least minNodeSpacing between consecutive nodes.
	// This is the correct noise-suppression approach: skip nodes until the
	// cumulative distance from the last kept node exceeds the threshold.
	// After resampling, merge_or_split indices into the original array are no
	// longer valid, so that correction is omitted; GetAverageCurvatures smooths
	// any residual spike at junction nodes.
	positions = SubsamplePositions(positions, minNodeSpacing)

	curvatures, err := GetCurvatures(positions)
	if err != nil {
		return []m.Curvature{}, errors.Wrap(err, "could not get curvatures from points")
	}

	average_curvatures, err := GetAverageCurvatures(curvatures)
	if err != nil {
		return []m.Curvature{}, errors.Wrap(err, "could not get average curvatures from curvatures")
	}
	return average_curvatures, nil
}

type Velocity struct {
	Pos             m.Position
	Velocity        float64
	TriggerDistance float32
}

func GetTargetVelocities(curvatures []m.Curvature, previousTargets []Velocity) (velocities []Velocity) {
	velocities = make([]Velocity, len(curvatures))
	for i, curv := range curvatures {
		if curv.Curvature == 0 {
			continue
		}
		velocities[i].Velocity = math.Pow(float64(ms.Settings.MapCurveTargetLatA)/curv.Curvature, 1.0/2)
		velocities[i].Pos = curv.Pos
		for _, t := range previousTargets {
			if velocities[i].Pos.Equals(t.Pos) {
				velocities[i].TriggerDistance = t.TriggerDistance
			}
		}
	}
	return velocities
}

func GetAverageCurvatures(curvatures []m.Curvature) (average_curvatures []m.Curvature, err error) {
	if len(curvatures) < 3 {
		return []m.Curvature{}, errors.New("not enough curvatures to average")
	}

	average_curvatures = make([]m.Curvature, len(curvatures)-2)

	for i := 0; i < len(curvatures)-2; i++ {
		a := curvatures[i].Curvature
		b := curvatures[i+1].Curvature
		c := curvatures[i+2].Curvature
		al := curvatures[i].ArcLength
		bl := curvatures[i+1].ArcLength
		cl := curvatures[i+2].ArcLength

		if al+bl+cl == 0 {
			average_curvatures[i] = curvatures[i+2]
			continue
		}

		avg := m.Curvature{Pos: curvatures[i+1].Pos}
		avg.Curvature = (a*al + b*bl + c*cl) / (al + bl + cl)
		avg.ArcLength = (curvatures[i].ArcLength + curvatures[i+1].ArcLength + curvatures[i+2].ArcLength) / 3
		avg.Angle = (curvatures[i].Angle + curvatures[i+1].Angle + curvatures[i+2].Angle) / 3
		average_curvatures[i] = avg
	}

	return average_curvatures, nil
}

func GetCurvatures(positions []m.Position) (curvatures []m.Curvature, err error) {
	if len(positions) < 3 {
		return []m.Curvature{}, errors.New(fmt.Sprintf("not enough points to calculate curvatures. len(points): %d", len(positions)))
	}
	curvatures = make([]m.Curvature, len(positions)-2)
	for i := 0; i < len(positions)-2; i++ {
		curvatures[i] = m.CalculateCurvature(positions[i], positions[i+1], positions[i+2])
	}
	return curvatures, nil
}
