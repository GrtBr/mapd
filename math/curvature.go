package math

import (
	m "math"
)

type Curvature struct {
	Curvature, ArcLength, Angle float64
	Pos                         Position
	// KIdx is the surviving original k-index from GetCurvatures' loop.
	// It lets downstream code map a Curvature entry back to its position
	// in the input slice even when GetCurvatures silently filtered some k
	// (e.g. all triplet widths exceeded maxChordSpacing). Zero is a valid
	// k-index, so callers must set it explicitly — CalculateCurvature
	// returns KIdx=0 (zero-init); GetCurvatures overwrites it before
	// appending.
	KIdx int
}

func CalculateCurvature(a Position, b Position, c Position) Curvature {
	lengthA := float64(a.DistanceTo(b))
	lengthB := float64(a.DistanceTo(c))
	lengthC := float64(b.DistanceTo(c))

	sp := (lengthA + lengthB + lengthC) / 2

	discriminant := sp * (sp - lengthA) * (sp - lengthB) * (sp - lengthC)
	if discriminant < 0 {
		discriminant = 0
	}
	area := m.Sqrt(discriminant)

	lengthProd := lengthA * lengthB * lengthC
	if lengthProd == 0 {
		return Curvature{Pos: b}
	}

	res := Curvature{Pos: b}
	res.Curvature = (4 * area) / lengthProd
	radius := 1.0 / res.Curvature

	num := (m.Pow(radius, 2)*2 - m.Pow(lengthB, 2))
	den := (2 * m.Pow(radius, 2))
	res.Angle = m.Acos(num / den)

	res.ArcLength = radius * res.Angle

	return res
}
