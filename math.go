package main

import (
	"fmt"
	"math"

	"github.com/pkg/errors"
	m "pfeifer.dev/mapd/math"
	ms "pfeifer.dev/mapd/settings"
)

// maxChordSpacing caps the outer chord length for L⁴-weighted triplets.
// If the chord from positions[k-w] to positions[k+w] exceeds this, the width
// is skipped and no wider widths are tried (chords only grow with w).
// 150 m prevents a triplet spanning a junction or a long OSM gap.
const maxChordSpacing float32 = 150.0

// maxTripletWidth is the maximum half-width of the symmetric triplet.
// Widths 1, 2, 3 are tried; each contributes weight = chord⁴.
const maxTripletWidth int = 3

func GetStateCurvatures(state *State) ([]m.Curvature, error) {
	nodes := state.CurrentWay.Way.Nodes()
	num_points := len(nodes)
	all_nodes := [][]m.Position{nodes}
	all_nodes_direction := []bool{state.CurrentWay.OnWay.IsForward}
	for _, nextWay := range state.NextWays {
		nwNodes := nextWay.Way.Nodes()
		if len(nwNodes) > 0 {
			num_points += len(nwNodes) - 1
		}
		all_nodes = append(all_nodes, nwNodes)
		all_nodes_direction = append(all_nodes_direction, nextWay.IsForward)
	}

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

	// L⁴-weighted multi-scale curvature — see GetCurvatures for details.
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

// GetCurvatures computes one L⁴-weighted curvature per interior node.
// For each node k (as the fixed middle), it tries symmetric triplets at
// half-widths w = 1, 2, 3 using positions[k-w] and positions[k+w] as outer
// points. Each estimate is weighted by chord(k-w, k+w)⁴ — the precision-optimal
// (inverse-variance) weight because curvature noise scales as δ/chord².
// A single displaced OSM node is suppressed because the wider (cleaner) triplets
// dominate the blend. Genuine curves are preserved because all widths agree there.
// The search stops at the first width whose chord exceeds maxChordSpacing, preventing
// triplets from spanning junctions or large OSM gaps.
func GetCurvatures(positions []m.Position) (curvatures []m.Curvature, err error) {
	if len(positions) < 3 {
		return []m.Curvature{}, errors.New(fmt.Sprintf("not enough points to calculate curvatures. len(points): %d", len(positions)))
	}
	curvatures = make([]m.Curvature, 0, len(positions))
	for k := 1; k < len(positions)-1; k++ {
		var totalWeight, totalCurv float64
		var base m.Curvature
		for w := 1; w <= maxTripletWidth; w++ {
			i, j := k-w, k+w
			if i < 0 || j >= len(positions) {
				break
			}
			chord := float64(positions[i].DistanceTo(positions[j]))
			if chord > float64(maxChordSpacing) {
				break
			}
			c := m.CalculateCurvature(positions[i], positions[k], positions[j])
			if w == 1 {
				base = c // preserve Pos and ArcLength from the immediate neighbours
			}
			weight := chord * chord * chord * chord
			totalWeight += weight
			totalCurv += c.Curvature * weight
		}
		if totalWeight > 0 {
			base.Curvature = totalCurv / totalWeight
			curvatures = append(curvatures, base)
		}
	}
	return curvatures, nil
}
