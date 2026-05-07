package maps

import (
	"context"
	"fmt"
	"log/slog"
	"math"
	"os"
	"sort"
	"strconv"
	"strings"

	"capnproto.org/go/capnp/v3"
	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
	"github.com/pkg/errors"
	"pfeifer.dev/mapd/cereal/offline"
	m "pfeifer.dev/mapd/math"
	"pfeifer.dev/mapd/params"
	ms "pfeifer.dev/mapd/settings"
	"pfeifer.dev/mapd/utils"
)

// hazardIndex is a compact 1-byte representation of OSM node hazard tags.
// Using uint8 instead of string saves 15 bytes per node; for SA's ~150 M way
// nodes that's ~2.25 GB saved vs the string approach.
type hazardIndex uint8

const (
	hazardNone           hazardIndex = 0
	hazardStop           hazardIndex = 1
	hazardGiveWay        hazardIndex = 2
	hazardRoundabout     hazardIndex = 3
	hazardMiniRoundabout hazardIndex = 4
	hazardTurningCircle  hazardIndex = 5
	hazardTollBooth      hazardIndex = 6
	hazardLevelCrossing  hazardIndex = 7
	hazardRailwayCrossing hazardIndex = 8
	hazardTrafficCalming hazardIndex = 9
)

func hazardIndexFromString(s string) hazardIndex {
	switch s {
	case "stop":             return hazardStop
	case "give_way":         return hazardGiveWay
	case "roundabout":       return hazardRoundabout
	case "mini_roundabout":  return hazardMiniRoundabout
	case "turning_circle":   return hazardTurningCircle
	case "toll_booth":       return hazardTollBooth
	case "level_crossing":   return hazardLevelCrossing
	case "railway_crossing": return hazardRailwayCrossing
	case "traffic_calming":  return hazardTrafficCalming
	}
	return hazardNone
}

func (h hazardIndex) String() string {
	switch h {
	case hazardStop:             return "stop"
	case hazardGiveWay:          return "give_way"
	case hazardRoundabout:       return "roundabout"
	case hazardMiniRoundabout:   return "mini_roundabout"
	case hazardTurningCircle:    return "turning_circle"
	case hazardTollBooth:        return "toll_booth"
	case hazardLevelCrossing:    return "level_crossing"
	case hazardRailwayCrossing:  return "railway_crossing"
	case hazardTrafficCalming:   return "traffic_calming"
	}
	return ""
}

// TmpNode uses float32 lat/lon (saves 8 bytes vs float64) and hazardIndex
// (saves 15 bytes vs string). Total: 9 bytes vs 32 bytes — 72% smaller.
type TmpNode struct {
	Latitude  float32
	Longitude float32
	Hazard    hazardIndex
}

// nodeCoordEntry is a compact node coordinate record used during tile generation.
// Using float32 and a sorted slice instead of map[NodeID][2]float64 reduces peak
// RSS from ~7.8 GB to ~640 MB for the 40 M-node SA extract.
type nodeCoordEntry struct {
	id  int64
	lat float32
	lon float32
}
type TmpWay struct {
	Name             string
	Ref              string
	Hazard           string
	MaxSpeed         float64
	MaxSpeedForward  float64
	MaxSpeedBackward float64
	MaxSpeedAdvisory float64
	Lanes            uint8
	Box              m.Box
	OneWay           bool
	Nodes            []TmpNode
}

type Area struct {
	Box  m.Box
	Ways []TmpWay
}

func (a *Area) OverlapBox(overlap float64) m.Box {
	return m.Box{
		MinPos: m.NewPosition(a.Box.MinPos.Lat()-overlap, a.Box.MinPos.Lon()-overlap),
		MaxPos: m.NewPosition(a.Box.MaxPos.Lat()+overlap, a.Box.MaxPos.Lon()+overlap),
	}
}

type OfflineSettings struct {
	Box                m.Box
	OutputDirectory    string
	InputFile          string
	GenerateEmptyFiles bool
	Overlap            float64
}

var DEFAULT_SETTINGS = OfflineSettings{
	OutputDirectory: fmt.Sprintf("%s/offline", params.GetBaseOpPath()),
}

func EnsureOfflineMapsDirectories(s OfflineSettings) {
	if err := os.MkdirAll(s.OutputDirectory, 0o775); err != nil {
		slog.Warn("could not make offline maps directory", "error", err, "directory", s.OutputDirectory)
	}
}

// Creates a file for a specific bounding box
func GenerateBoundsFileName(a Area, s OfflineSettings) string {
	p := a.Box.GroupPos()
	dir := fmt.Sprintf("%s/%d/%d", s.OutputDirectory, int(p.Lat()), int(p.Lon()))
	return fmt.Sprintf("%s/%f_%f_%f_%f", dir, a.Box.MinPos.Lat(), a.Box.MinPos.Lon(), a.Box.MaxPos.Lat(), a.Box.MaxPos.Lon())
}

// Creates a file for a specific bounding box
func CreateBoundsDir(a Area, s OfflineSettings) error {
	p := a.Box.GroupPos()
	dir := fmt.Sprintf("%s/%d/%d", s.OutputDirectory, int(p.Lat()), int(p.Lon()))
	err := os.MkdirAll(dir, 0o775)
	return errors.Wrap(err, "could not create bounds directory")
}

// Generates bounding boxes for storing ways
func generateAreas() []Area {
	areas := make([]Area, int((361/ms.AREA_BOX_DEGREES)*(181/ms.AREA_BOX_DEGREES)))
	index := 0
	for i := float64(-90); i < 90; i += ms.AREA_BOX_DEGREES {
		for j := float64(-180); j < 180; j += ms.AREA_BOX_DEGREES {
			a := &areas[index]
			a.Box = m.Box{
				MinPos: m.NewPosition(i, j),
				MaxPos: m.NewPosition(i+ms.AREA_BOX_DEGREES, j+ms.AREA_BOX_DEGREES),
			}
			index += 1
		}
	}
	return areas
}

func GenerateOffline(s OfflineSettings) {
	slog.Info("Generating Offline Map")
	EnsureOfflineMapsDirectories(s)

	// nodeCollectionBox is the processing bbox extended by 1° on every side.
	// 1° of padding ensures nodes belonging to ways that straddle the bbox
	// boundary are still resolved correctly in pass 2.
	const nodeCollectPadDeg = 1.0
	nodeCollectionBox := s.Box.Overlap(nodeCollectPadDeg)

	areas := generateAreas()
	overlapBox := s.Box.Overlap(s.Overlap)
	relevantAreas := make([]*Area, 0, 512)
	for i := range areas {
		if overlapBox.Overlapping(areas[i].Box) {
			relevantAreas = append(relevantAreas, &areas[i])
		}
	}

	// ── Pass 1: collect node coordinates (bbox-filtered) ────────────────────
	// SkipWays=true means way blobs are skipped at the protobuf level (not just
	// filtered), so pass 1 is fast. FilterNode causes memory reuse for out-of-bbox
	// nodes inside the decoder goroutines — no heap allocation for skipped nodes.
	slog.Info("Pass 1: collecting node coordinates")
	taggedNodes := make(map[osm.NodeID]string)
	hazardGrid  := make(map[hazardCellKey][]hazardPosEntry)
	var nodeCoords []nodeCoordEntry

	file1, err := os.Open(s.InputFile)
	if err != nil {
		slog.Error("could not open map pbf file", "error", err)
		panic("failed to read maps, exiting")
	}
	scanner1 := osmpbf.New(context.Background(), file1, 2)
	scanner1.SkipWays = true
	scanner1.SkipRelations = true
	scanner1.FilterNode = func(n *osm.Node) bool {
		return nodeCollectionBox.PosInside(m.NewPosition(n.Lat, n.Lon))
	}
	for scanner1.Scan() {
		node := scanner1.Object().(*osm.Node)
		if h := extractNodeHazard(node); h != "" {
			taggedNodes[node.ID] = h
			// Also add to spatial grid for proximity matching of standalone hazard
			// nodes (e.g. highway=stop) that are never referenced by road ways.
			hi := hazardIndexFromString(h)
			key := hazardCellKey{
				lat: int32(float64(node.Lat) / hazardGridCell),
				lon: int32(float64(node.Lon) / hazardGridCell),
			}
			hazardGrid[key] = append(hazardGrid[key], hazardPosEntry{
				lat: float32(node.Lat), lon: float32(node.Lon), hazard: hi,
			})
		}
		nodeCoords = append(nodeCoords, nodeCoordEntry{
			id:  int64(node.ID),
			lat: float32(node.Lat),
			lon: float32(node.Lon),
		})
	}
	if err := scanner1.Err(); err != nil {
		slog.Error("node scan error (pass 1)", "error", err)
		panic("failed to read maps, exiting")
	}
	scanner1.Close()
	file1.Close()

	sort.Slice(nodeCoords, func(i, j int) bool { return nodeCoords[i].id < nodeCoords[j].id })
	slog.Info("Pass 1 complete", "nodes", len(nodeCoords), "taggedNodes", len(taggedNodes))

	// ── Pass 2: scan ways, FilterWay skips out-of-bbox ways ─────────────────
	// FilterWay runs inside decoder goroutines BEFORE osm.Way heap allocation.
	// When it returns false the library reuses the struct — no alloc.
	// This is the critical fix for the 14 GB OOM: the SA PBF has ~5 M ways and
	// all of them were being allocated before any bbox filtering could happen.
	slog.Info("Pass 2: scanning ways")
	file2, err := os.Open(s.InputFile)
	if err != nil {
		slog.Error("could not open map pbf file (pass 2)", "error", err)
		panic("failed to read maps, exiting")
	}
	scanner2 := osmpbf.New(context.Background(), file2, 2)
	scanner2.SkipNodes = true
	scanner2.SkipRelations = true
	scanner2.FilterWay = func(w *osm.Way) bool {
		for _, n := range w.Nodes {
			lat, lon := lookupNodeCoord(nodeCoords, int64(n.ID))
			if lat != 0 || lon != 0 {
				return true
			}
		}
		return false
	}
	for scanner2.Scan() {
		way := scanner2.Object().(*osm.Way)
		if len(way.Nodes) <= 1 {
			continue
		}
		tags := way.TagMap()
		lanes, _ := strconv.ParseUint(tags["lanes"], 10, 8)
		tmpWay := TmpWay{
			Nodes:            make([]TmpNode, len(way.Nodes)),
			Name:             tags["name"],
			Ref:              tags["ref"],
			Hazard:           tags["hazard"],
			MaxSpeed:         ParseMaxSpeed(tags["maxspeed"]),
			MaxSpeedForward:  ParseMaxSpeed(tags["maxspeed:forward"]),
			MaxSpeedBackward: ParseMaxSpeed(tags["maxspeed:backward"]),
			MaxSpeedAdvisory: ParseMaxSpeed(tags["maxspeed:advisory"]),
			Lanes:            uint8(lanes),
			OneWay:           tags["oneway"] == "yes",
		}

		minLat := float32(90)
		minLon := float32(180)
		maxLat := float32(-90)
		maxLon := float32(-180)
		for i, n := range way.Nodes {
			lat32, lon32 := lookupNodeCoord(nodeCoords, int64(n.ID))
			if lat32 < minLat {
				minLat = lat32
			}
			if lon32 < minLon {
				minLon = lon32
			}
			if lat32 > maxLat {
				maxLat = lat32
			}
			if lon32 > maxLon {
				maxLon = lon32
			}
			tmpWay.Nodes[i].Latitude = lat32
			tmpWay.Nodes[i].Longitude = lon32
			tmpWay.Nodes[i].Hazard = hazardIndexFromString(taggedNodes[n.ID])
		}
		tmpWay.Box.MinPos = m.NewPosition(float64(minLat), float64(minLon))
		tmpWay.Box.MaxPos = m.NewPosition(float64(maxLat), float64(maxLon))

		// Proximity fallback for the first and last nodes of each way.
		// Stop-sign nodes (highway=stop) are standalone OSM objects that are never
		// members of road ways, so the exact ID match above always returns hazardNone
		// for them.  Search for the nearest hazard node within hazardProximityM metres
		// and snap it to the endpoint if found.  Only endpoints matter because the
		// runtime's NodeHazardAtJunction reads nodes[0] (forward) or nodes[last]
		// (reverse) — mid-road nodes are never consulted for junction hazards.
		if n := len(tmpWay.Nodes); n > 0 {
			first := &tmpWay.Nodes[0]
			if first.Hazard == hazardNone {
				first.Hazard = nearbyHazard(hazardGrid, first.Latitude, first.Longitude)
			}
			if n > 1 {
				last := &tmpWay.Nodes[n-1]
				if last.Hazard == hazardNone {
					last.Hazard = nearbyHazard(hazardGrid, last.Latitude, last.Longitude)
				}
			}
		}

		for _, area := range relevantAreas {
			if tmpWay.Box.Overlapping(area.OverlapBox(s.Overlap)) {
				area.Ways = append(area.Ways, tmpWay)
			}
		}
	}
	if err := scanner2.Err(); err != nil {
		slog.Error("way scan error (pass 2)", "error", err)
		panic("failed to read maps, exiting")
	}
	scanner2.Close()
	file2.Close()

	nodeCoords = nil // free node coordinate cache before write phase

	slog.Info("Finding Bounds")
	for _, area := range areas {
		if !overlapBox.Contains(area.Box) {
			continue
		}

		if len(area.Ways) == 0 && !s.GenerateEmptyFiles {
			continue
		}

		arena := capnp.MultiSegment([][]byte{})
		msg, seg, err := capnp.NewMessage(arena)
		if err != nil {
			slog.Error("could not create capnp arena for offline data", "error", err)
			panic("unexpected capnp error, exiting")
		}
		rootOffline, err := offline.NewRootOffline(seg)
		if err != nil {
			slog.Error("could not create capnp root for offline data", "error", err)
			panic("unexpected capnp error, exiting")
		}

		slog.Info("Writing Area")
		ways, err := rootOffline.NewWays(int32(len(area.Ways)))
		if err != nil {
			slog.Error("could not create ways in offline data", "error", err)
			panic("unexpected capnp error, exiting")
		}
		rootOffline.SetMinLat(area.Box.MinPos.Lat())
		rootOffline.SetMinLon(area.Box.MinPos.Lon())
		rootOffline.SetMaxLat(area.Box.MaxPos.Lat())
		rootOffline.SetMaxLon(area.Box.MaxPos.Lon())
		rootOffline.SetOverlap(s.Overlap)
		for i, way := range area.Ways {
			w := ways.At(i)
			w.SetMinLat(way.Box.MinPos.Lat())
			w.SetMinLon(way.Box.MinPos.Lon())
			w.SetMaxLat(way.Box.MaxPos.Lat())
			w.SetMaxLon(way.Box.MaxPos.Lon())
			err := w.SetName(way.Name)
			if err != nil {
				slog.Error("could not set way name", "error", err)
				panic("unexpected capnp error, exiting")
			}
			err = w.SetRef(way.Ref)
			if err != nil {
				slog.Error("could not set way ref", "error", err)
				panic("unexpected capnp error, exiting")
			}
			err = w.SetHazard(way.Hazard)
			if err != nil {
				slog.Error("could not set way hazard", "error", err)
				panic("unexpected capnp error, exiting")
			}
			w.SetMaxSpeed(way.MaxSpeed)
			w.SetMaxSpeedForward(way.MaxSpeedForward)
			w.SetMaxSpeedBackward(way.MaxSpeedBackward)
			w.SetAdvisorySpeed(way.MaxSpeedAdvisory)
			w.SetLanes(way.Lanes)
			w.SetOneWay(way.OneWay)
			nodes, err := w.NewNodes(int32(len(way.Nodes)))
			if err != nil {
				slog.Error("could not create way nodes", "error", err)
				panic("unexpected capnp error, exiting")
			}
			for j, node := range way.Nodes {
				n := nodes.At(j)
				n.SetLatitude(float64(node.Latitude))
				n.SetLongitude(float64(node.Longitude))
				if node.Hazard != hazardNone {
					if err := n.SetHazard(node.Hazard.String()); err != nil {
						slog.Error("could not set node hazard", "error", err)
					}
				}
			}
		}

		data, err := msg.MarshalPacked()
		if err != nil {
			slog.Error("could not marshal offline data", "error", err)
			panic("unexpected capnp error, exiting")
		}
		err = CreateBoundsDir(area, s)
		if err != nil {
			slog.Error("could not create bounds directory", "error", err)
			panic("unexpected file error, exiting")
		}
		err = os.WriteFile(GenerateBoundsFileName(area, s), data, 0o644)
		if err != nil {
			slog.Error("could not write offline data to file", "error", err)
			panic("unexpected file error, exiting")
		}
	}
	f, err := os.Open(s.OutputDirectory)
	if err != nil {
		slog.Error("could not open bounds directory", "error", err)
		panic("unexpected file error, exiting")
	}
	err = f.Sync()
	if err != nil {
		slog.Error("could not fsync bounds directory", "error", err)
		panic("unexpected file error, exiting")
	}
	err = f.Close()
	if err != nil {
		slog.Error("could not close bounds directory", "error", err)
		panic("unexpected file error, exiting")
	}

	slog.Info("Done Generating Offline Map")
}

var AREAS = generateAreas()

func FindWaysAroundPosition(pos m.Position) (Offline, error) {
	for _, area := range AREAS {
		inBox := area.Box.PosInside(pos)
		if inBox {
			boundsName := GenerateBoundsFileName(area, DEFAULT_SETTINGS)
			slog.Info("Loading bounds file", "filename", boundsName)
			data, err := os.ReadFile(boundsName)
			o := ReadOffline(data)
			if !o.Loaded {
				area := Area{}
				areas := generateAreas()
				for _, a := range areas {
					if a.Box.PosInside(pos) {
						area = a
					}
				}
				o.box.Set(area.Box)
			}
			return o, errors.Wrap(err, "could not read current offline data file")
		}
	}
	area := Area{}
	areas := generateAreas()
	for _, a := range areas {
		if a.Box.PosInside(pos) {
			area = a
		}
	}
	cBox := utils.Curry[m.Box]{}
	cBox.Set(area.Box)
	return Offline{
		Loaded: false,
		box:    cBox,
	}, nil
}

func ParseMaxSpeed(maxspeed string) float64 {
	splitSpeed := strings.Split(maxspeed, " ")
	if len(splitSpeed) == 0 {
		return 0
	}

	numeric, err := strconv.ParseUint(splitSpeed[0], 10, 64)
	if err != nil {
		return 0
	}

	if len(splitSpeed) == 1 {
		return 0.277778 * float64(numeric)
	}

	if splitSpeed[1] == "kph" || splitSpeed[1] == "km/h" || splitSpeed[1] == "kmh" {
		return 0.277778 * float64(numeric)
	} else if splitSpeed[1] == "mph" {
		return 0.44704 * float64(numeric)
	} else if splitSpeed[1] == "knots" {
		return 0.514444 * float64(numeric)
	}

	return 0
}

// lookupNodeCoord binary-searches the sorted nodeCoords slice for a node ID and
// returns its lat/lon as float32. Returns (0, 0) if the node is not found.
func lookupNodeCoord(coords []nodeCoordEntry, id int64) (float32, float32) {
	idx := sort.Search(len(coords), func(i int) bool { return coords[i].id >= id })
	if idx < len(coords) && coords[idx].id == id {
		return coords[idx].lat, coords[idx].lon
	}
	return 0, 0
}

// hazardCellKey indexes a 0.001° (~111 m) grid cell for the proximity index.
type hazardCellKey struct{ lat, lon int32 }

// hazardPosEntry stores the position and hazard type of a standalone hazard node.
type hazardPosEntry struct {
	lat, lon float32
	hazard   hazardIndex
}

const hazardGridCell = 0.001 // degrees per grid cell (~111 m)
const hazardProximityM = 30.0 // max snap distance in metres

// nearbyHazard searches the 3×3 cell neighbourhood around (lat, lon) and returns
// the hazardIndex of the closest hazard node within hazardProximityM metres.
// Returns hazardNone if nothing is within threshold.
func nearbyHazard(grid map[hazardCellKey][]hazardPosEntry, lat, lon float32) hazardIndex {
	cy := int32(float64(lat) / hazardGridCell)
	cx := int32(float64(lon) / hazardGridCell)
	bestD2 := float64(hazardProximityM * hazardProximityM)
	best := hazardNone
	cosLat := math.Cos(float64(lat) * math.Pi / 180.0)
	for dy := int32(-1); dy <= 1; dy++ {
		for dx := int32(-1); dx <= 1; dx++ {
			for _, hp := range grid[hazardCellKey{cy + dy, cx + dx}] {
				dlat := float64(lat-hp.lat) * 111_320
				dlon := float64(lon-hp.lon) * 111_320 * cosLat
				d2 := dlat*dlat + dlon*dlon
				if d2 < bestD2 {
					bestD2 = d2
					best = hp.hazard
				}
			}
		}
	}
	return best
}

// extractNodeHazard returns a hazard tag string for OSM nodes that require
// speed reduction: stop signs, give-way, crossings, traffic calming, etc.
// Returns "" if the node has no recognised hazard tag.
func extractNodeHazard(node *osm.Node) string {
	tags := node.Tags
	switch tags.Find("highway") {
	case "stop":
		return "stop"
	case "give_way":
		return "give_way"
	case "turning_circle":
		return "turning_circle"
	case "mini_roundabout":
		return "mini_roundabout"
	}
	if tags.Find("junction") == "roundabout" {
		return "roundabout"
	}
	if tags.Find("barrier") == "toll_booth" {
		return "toll_booth"
	}
	switch tags.Find("railway") {
	case "level_crossing":
		return "level_crossing"
	case "railway_crossing":
		return "railway_crossing"
	}
	tc := tags.Find("traffic_calming")
	if tc != "" {
		switch tc {
		case "painted_island", "surface", "marking":
			return ""
		}
		return "traffic_calming"
	}
	return ""
}
