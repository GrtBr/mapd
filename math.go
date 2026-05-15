package main

import (
	"fmt"
	"math"

	"github.com/pkg/errors"
	"pfeifer.dev/mapd/maps"
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

// gaussSigma is the arc-length standard deviation (metres) of the Gaussian
// kernel used to pre-smooth OSM node positions before curvature computation.
// σ=10 m: suppresses isolated displaced nodes (they contribute ~1/5 of error
// among neighbours at 20 m spacing) while preserving tight curves (r≥30 m
// loses <10% curvature vs 41% at σ=25 m).
const gaussSigma float64 = 10.0

// smoothPositions returns a new slice where each position is replaced by a
// Gaussian-weighted centroid of all positions in the way, with weights
// decaying by arc-length distance from that node (σ=gaussSigma).
// This suppresses isolated OSM node positioning errors before curvature is computed.
func smoothPositions(positions []m.Position) []m.Position {
	n := len(positions)
	out := make([]m.Position, n)

	arcLen := make([]float64, n)
	for i := 1; i < n; i++ {
		arcLen[i] = arcLen[i-1] + float64(positions[i-1].DistanceTo(positions[i]))
	}

	inv2sig2 := 1.0 / (2.0 * gaussSigma * gaussSigma)
	for k := 0; k < n; k++ {
		var wLat, wLon, wSum float64
		for i := 0; i < n; i++ {
			d := arcLen[k] - arcLen[i]
			w := math.Exp(-d * d * inv2sig2)
			wLat += positions[i].Lat() * w
			wLon += positions[i].Lon() * w
			wSum += w
		}
		out[k] = m.NewPosition(wLat/wSum, wLon/wSum)
	}
	return out
}

// readWayCurvatures reads the per-node stored curvature side-channel for one
// way from the raw capnp Coordinates list. If the capnp accessor errors, it
// returns a zero-filled slice of the same length as the parallel m.Position
// slice — zero means "no stored κ, fall back to live compute" downstream.
func readWayCurvatures(w maps.Way, nodeCount int) []float64 {
	kappa := make([]float64, nodeCount)
	rawNodes, err := w.Way.Nodes()
	if err != nil {
		return kappa
	}
	// Defensive: raw capnp length must match the wrapper's []m.Position length.
	// If it doesn't (shouldn't happen since both come from the same capnp object),
	// the missing entries stay 0.0 → live fallback.
	n := rawNodes.Len()
	if n > nodeCount {
		n = nodeCount
	}
	for j := 0; j < n; j++ {
		kappa[j] = rawNodes.At(j).Curvature()
	}
	return kappa
}

func GetStateCurvatures(state *State) ([]m.Curvature, error) {
	nodes := state.CurrentWay.Way.Nodes()
	num_points := len(nodes)
	all_nodes := [][]m.Position{nodes}
	all_nodes_curvatures := [][]float64{readWayCurvatures(state.CurrentWay.Way, len(nodes))}
	all_nodes_direction := []bool{state.CurrentWay.OnWay.IsForward}
	for _, nextWay := range state.NextWays {
		nwNodes := nextWay.Way.Nodes()
		if len(nwNodes) > 0 {
			num_points += len(nwNodes) - 1
		}
		all_nodes = append(all_nodes, nwNodes)
		all_nodes_curvatures = append(all_nodes_curvatures, readWayCurvatures(nextWay.Way, len(nwNodes)))
		all_nodes_direction = append(all_nodes_direction, nextWay.IsForward)
	}

	// positions and storedKappa are built in lockstep — one index, one OSM node.
	// storedKappa[i] > 0 means the tile contained a precomputed κ for positions[i]
	// (Phase 1 precompute, Decision 3). storedKappa[i] == 0 → live-fallback.
	positions := make([]m.Position, num_points)
	storedKappa := make([]float64, num_points)

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
		storedKappa[i] = all_nodes_curvatures[all_nodes_idx][index]

		nodes_idx += 1
		if nodes_idx == len(all_nodes[all_nodes_idx]) || (nodes_idx == len(all_nodes[all_nodes_idx])-1 && all_nodes_idx > 0) {
			all_nodes_idx += 1
			nodes_idx = 0
		}
	}

	if len(positions) != len(storedKappa) {
		return []m.Curvature{}, fmt.Errorf("storedKappa/positions length mismatch: pos=%d kap=%d", len(positions), len(storedKappa))
	}

	// L⁴-weighted multi-scale curvature — see GetCurvatures for details.
	curvatures, err := GetCurvatures(positions)
	if err != nil {
		return []m.Curvature{}, errors.Wrap(err, "could not get curvatures from points")
	}

	// Prefer stored κ over live where present. curvatures[i].KIdx is the
	// original k-index from GetCurvatures' loop — robust against silent
	// filtering (e.g. all triplet widths exceeded maxChordSpacing for some
	// k, so that entry was never appended). Mirrors Phase 1 Bug B fix on
	// the Python side. Override Curvature only — keep Pos / ArcLength /
	// Angle from the live pipeline (they're geometry-derived).
	for i := range curvatures {
		k := curvatures[i].KIdx
		if k >= 0 && k < len(storedKappa) && storedKappa[k] > 0 {
			curvatures[i].Curvature = storedKappa[k]
		}
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
		// Thread KIdx from the centred (middle) triplet entry — same convention
		// as Pos. Lets downstream code map the averaged entry back to its
		// original position in GetStateCurvatures' positions slice.
		avg.KIdx = curvatures[i+1].KIdx
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
	positions = smoothPositions(positions)
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
			// KIdx carries the original k-index so downstream code can map
			// curvatures[i] back to positions[KIdx] even when this loop
			// silently skipped k's whose all-widths chord exceeded
			// maxChordSpacing.
			base.KIdx = k
			curvatures = append(curvatures, base)
		}
	}
	return curvatures, nil
}
