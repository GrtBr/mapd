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

// Per-highway Gaussian σ (metres of arc-length). Mirrors generate_tiles_ver2.py's
// _sigma_for_highway so the LIVE fallback uses the same smoothing radius as the
// stored κ on the same way. Without this match, κ jumps at every node where the
// runtime falls back from stored to live because the smoothing widths differ.
//
//   sigmaLow  (15 m) — residential / unclassified: sparser OSM nodes (~20–30 m
//             spacing) with genuinely tight geometry to preserve.
//   sigmaHigh (30 m) — motorway / trunk / primary / secondary / tertiary family
//             (and links, living_street, road, service): densely traced
//             (~10 m spacing), needs heavier smoothing to suppress hand-tracing
//             noise while real curves stay above r ≥ ~60 m.
//
// gaussSigma is kept as a legacy fallback name (used only when a way's highway
// tag is missing or the tile predates the highway @14 field).
const sigmaLow float64 = 15.0
const sigmaHigh float64 = 30.0
const gaussSigma float64 = sigmaLow // fallback when highway tag is absent

// sigmaForHighway returns the smoothing σ for a way of OSM highway tag `hw`.
// Mirrors the Python helper of the same name in generate_tiles_ver2.py.
func sigmaForHighway(hw string) float64 {
	switch hw {
	case "residential", "unclassified":
		return sigmaLow
	}
	if hw == "" {
		return gaussSigma
	}
	return sigmaHigh
}

// tilesOnlyDebug — TEMPORARY DEBUG FLAG. When true, GetStateCurvatures uses
// ONLY the precomputed tile curvature: storedKappa is assigned at every node
// (including 0), so the live GetCurvatures fallback is fully discarded. Lets
// on-road behaviour faithfully reflect the deployed tile set while debugging
// tile quality. Set back to false (and rebuild) when finished.
const tilesOnlyDebug = false

// smoothPositions returns a new slice where each position is replaced by a
// Gaussian-weighted centroid of all positions in the way, with weights
// decaying by arc-length distance from that node (standard deviation `sigma`).
// This suppresses isolated OSM node positioning errors before curvature is
// computed. `sigma` is per-highway (see sigmaForHighway).
func smoothPositions(positions []m.Position, sigma float64) []m.Position {
	n := len(positions)
	out := make([]m.Position, n)

	arcLen := make([]float64, n)
	for i := 1; i < n; i++ {
		arcLen[i] = arcLen[i-1] + float64(positions[i-1].DistanceTo(positions[i]))
	}

	inv2sig2 := 1.0 / (2.0 * sigma * sigma)
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
	all_nodes_sigma := []float64{sigmaForHighway(state.CurrentWay.Way.Highway())}
	for _, nextWay := range state.NextWays {
		nwNodes := nextWay.Way.Nodes()
		if len(nwNodes) > 0 {
			num_points += len(nwNodes) - 1
		}
		all_nodes = append(all_nodes, nwNodes)
		all_nodes_curvatures = append(all_nodes_curvatures, readWayCurvatures(nextWay.Way, len(nwNodes)))
		all_nodes_direction = append(all_nodes_direction, nextWay.IsForward)
		all_nodes_sigma = append(all_nodes_sigma, sigmaForHighway(nextWay.Way.Highway()))
	}

	// positions and storedKappa are built in lockstep — one index, one OSM node.
	// way_starts[k] records the offset in positions[] at which way k starts
	// contributing nodes (after the shared-junction dedup that the loop below
	// applies). way_starts has length len(all_nodes)+1; the trailing entry is
	// num_points so per-way slicing is uniform.
	positions := make([]m.Position, num_points)
	storedKappa := make([]float64, num_points)
	way_starts := make([]int, len(all_nodes)+1)
	way_starts[0] = 0

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
			if all_nodes_idx < len(all_nodes) {
				way_starts[all_nodes_idx] = i + 1
			}
		}
	}
	way_starts[len(all_nodes)] = num_points

	if len(positions) != len(storedKappa) {
		return []m.Curvature{}, fmt.Errorf("storedKappa/positions length mismatch: pos=%d kap=%d", len(positions), len(storedKappa))
	}

	// Group consecutive same-σ ways and run the full L⁴ + override + 3-pt MA
	// pipeline once PER GROUP. Mirrors generate_tiles_ver2.py's chain model:
	// a chain terminates at a σ-class boundary, each chain is smoothed with its
	// own σ, and the MA window doesn't bleed across the boundary. Without this,
	// the live fallback uses one σ for the whole CurrentWay+NextWays slice and
	// produces κ that's inconsistent with the stored κ on either side.
	var result []m.Curvature
	groupStart := 0
	for i := 1; i <= len(all_nodes); i++ {
		// Close a group at the end OR when the next way's σ-class differs.
		boundary := i == len(all_nodes) || all_nodes_sigma[i] != all_nodes_sigma[groupStart]
		if !boundary {
			continue
		}
		posStart := way_starts[groupStart]
		posEnd := way_starts[i]
		groupSigma := all_nodes_sigma[groupStart]

		if posEnd-posStart >= 3 {
			groupPositions := positions[posStart:posEnd]
			curvs, err := GetCurvatures(groupPositions, groupSigma)
			if err == nil {
				// Remap local KIdx → global positions[] index, then apply the
				// "prefer stored" override (or, in tilesOnlyDebug, assign stored
				// unconditionally — including 0 — so the live fallback is
				// discarded). KIdx flows through GetAverageCurvatures via the
				// centred-triplet convention.
				for j := range curvs {
					localK := curvs[j].KIdx
					globalK := localK + posStart
					curvs[j].KIdx = globalK
					if globalK >= 0 && globalK < len(storedKappa) {
						if tilesOnlyDebug || storedKappa[globalK] > 0 {
							curvs[j].Curvature = storedKappa[globalK]
						}
					}
				}
				if len(curvs) >= 3 {
					avgCurvs, err := GetAverageCurvatures(curvs)
					if err == nil {
						result = append(result, avgCurvs...)
					}
				}
			}
		}
		groupStart = i
	}

	return result, nil
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
func GetCurvatures(positions []m.Position, sigma float64) (curvatures []m.Curvature, err error) {
	if len(positions) < 3 {
		return []m.Curvature{}, errors.New(fmt.Sprintf("not enough points to calculate curvatures. len(points): %d", len(positions)))
	}
	positions = smoothPositions(positions, sigma)
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
