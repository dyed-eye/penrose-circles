package main

import (
	"container/list"
	"fmt"
	"io"
	"math"
	"os"
	"sort"
	"strings"
	"time"

	"github.com/jbeda/geom"
)

// Arc drawing functions/strategies
type ArcFuncType int

const (
	ARC_FUNC_CIRCULAR ArcFuncType = iota
	ARC_FUNC_BEZIER
)

// Tunable constants for output
const (
	DEFAULT_STYLE                 = "stroke-width: 0.002; stroke-linecap: round; fill: none"
	DOUBLE_STROKE_OFFSET          = 0.002
	CUT_STYLE                     = "stroke: black"
	MARK1_STYLE                   = "stroke: green"
	MARK2_STYLE                   = "stroke: red"
	DEFLATE_LEVEL                 = 10
	SQUISH_ARC_FUNC               = ARC_FUNC_CIRCULAR
	SQUISH_ARC_FACTOR             = 0.9
	SQUISH_ARC_BEZIER_ROUNDESS_A  = 0.15
	SQUISH_ARC_BEZIER_ROUNDESS_B  = 0.1
	DEBUG_MODE                    = false // Set to true for verbose output
	OPTIMIZE_MARK_TRAVEL          = false // Set to true to optimize mark path travel distance (slower)
	ENABLE_FAST_PATH_OPTIMIZATION = true  // Use spatial hashing for faster path optimization
)

// Mathematical constants for generating Penrose decompositions.
const (
	C1 = math.Phi - 1.0
	C2 = 2.0 - math.Phi
)

// //////////////////////////////////////////////////////////////////////////
// SVG serialization helper
type SVG struct {
	writer io.Writer
}

func NewSVG(w io.Writer) *SVG {
	return &SVG{w}
}

func (svg *SVG) printf(format string, a ...interface{}) (n int, errno error) {
	return fmt.Fprintf(svg.writer, format, a...)
}

// BUGBUG: not quoting aware
func extraparams(s []string) string {
	ep := ""
	for i := 0; i < len(s); i++ {
		if strings.Index(s[i], "=") > 0 {
			ep += (s[i]) + " "
		} else if len(s[i]) > 0 {
			ep += fmt.Sprintf("style='%s' ", s[i])
		}
	}
	return ep
}

func onezero(b bool) string {
	if b {
		return "1"
	}
	return "0"
}

func (svg *SVG) Start(viewBox geom.Rect, s ...string) {
	svg.printf(`<?xml version="1.0"?>
<svg version="1.1"
     viewBox="%f %f %f %f"
     xmlns="http://www.w3.org/2000/svg" %s>
`, viewBox.Min.X, viewBox.Min.Y, viewBox.Width(), viewBox.Height(), extraparams(s))
}

func (svg *SVG) End() {
	svg.printf("</svg>\n")
}

func (svg *SVG) Line(p1 geom.Coord, p2 geom.Coord, s ...string) {
	svg.printf("<line x1='%f' y1='%f' x2='%f' y2='%f' %s/>\n", p1.X, p1.Y, p2.X, p2.Y, extraparams(s))
}

func (svg *SVG) Circle(c geom.Coord, r float64, s ...string) {
	svg.printf("<circle cx='%f' cy='%f' r='%f' %s/>\n", c.X, c.Y, r, extraparams(s))
}

func (svg *SVG) CircularArc(p1, p2 geom.Coord, r float64, largeArc, sweep bool, s ...string) {
	svg.printf("<path d='M%f,%f A%f,%f 0 %s,%s %f,%f' %s/>\n",
		p1.X, p1.Y, r, r, onezero(largeArc), onezero(sweep), p2.X, p2.Y, extraparams(s))
}

func (svg *SVG) QuadBezier(p1 geom.Coord, ctrl1 geom.Coord, p2 geom.Coord, s ...string) {
	svg.printf("<path d='M%f,%f Q%f,%f %f,%f' %s/>\n",
		p1.X, p1.Y, ctrl1.X, ctrl1.Y, p2.X, p2.Y, extraparams(s))
}

func (svg *SVG) CubicBezier(p1, ctrl1, ctrl2, p2 geom.Coord, s ...string) {
	svg.printf("<path d='M%f,%f C%f,%f %f,%f %f,%f' %s/>\n",
		p1.X, p1.Y, ctrl1.X, ctrl1.Y, ctrl2.X, ctrl2.Y, p2.X, p2.Y, extraparams(s))
}

func (svg *SVG) StartPath(p1 geom.Coord, s ...string) {
	svg.printf("<path %sd='M%f,%f", extraparams(s), p1.X, p1.Y)
}

func (svg *SVG) EndPath() {
	svg.printf("'/>\n")
}

func (svg *SVG) PathLineTo(p geom.Coord) {
	svg.printf("\n  L%f,%f", p.X, p.Y)
}

func (svg *SVG) PathCircularArcTo(p geom.Coord, r float64, largeArc, sweep bool) {
	svg.printf("\n  A%f,%f 0 %s,%s %f,%f", r, r, onezero(largeArc), onezero(sweep), p.X, p.Y)
}

func (svg *SVG) PathQuadBezierTo(p, ctrl1 geom.Coord) {
	svg.printf("\n  Q%f,%f, %f,%f", ctrl1.X, ctrl1.Y, p.X, p.Y)
}

func (svg *SVG) PathCubicBezierTo(p, ctrl1, ctrl2 geom.Coord) {
	svg.printf("\n  C%f,%f, %f,%f %f,%f", ctrl1.X, ctrl1.Y, ctrl2.X, ctrl2.Y, p.X, p.Y)
}

////////////////////////////////////////////////////////////////////////////
// Math/Geometry Helpers

// Comparing floating point sucks.  This is probably wrong in the general case
// but is good enough for this application.  Don't assume I know what I'm
// doing here.  I'm pulling stuff out of my butt.
const FLOAT_EQUAL_THRESH = 0.00000001

func FloatAlmostEqual(a, b float64) bool {
	return math.Abs(a-b) < FLOAT_EQUAL_THRESH
}

func AlmostEqualsCoord(a, b geom.Coord) bool {
	return FloatAlmostEqual(a.X, b.X) && FloatAlmostEqual(a.Y, b.Y)
}

////////////////////////////////////////////////////////////////////////////
// Decomposed Penrose Geometry
//
// This stuff deals with pre-styled lines and arcs and is tailored to Penrose
// tiling needs.  This could be a generic retained mode 2D model thing but it
// is more quick and dirty for now.

// +++ Path
type PathSegment interface {
	P1() *geom.Coord
	P2() *geom.Coord
	Reverse()
	PathDraw(svg *SVG)
	Length() float64
}

type Path struct {
	segs *list.List
}

func (me *Path) PushFront(seg PathSegment) {
	if me.segs == nil {
		me.segs = new(list.List)
	}
	me.segs.PushFront(seg)
}

func (me *Path) PushPathFront(path *Path) {
	if me.segs == nil {
		me.segs = new(list.List)
	}
	me.segs.PushFrontList(path.segs)
}

func (me *Path) PushBack(seg PathSegment) {
	if me.segs == nil {
		me.segs = new(list.List)
	}
	me.segs.PushBack(seg)
}

func (me *Path) PushPathBack(path *Path) {
	if me.segs == nil {
		me.segs = new(list.List)
	}
	me.segs.PushBackList(path.segs)
}

func (me *Path) Reverse() {
	newSegs := new(list.List)
	for e := me.segs.Front(); e != nil; e = e.Next() {
		e.Value.(PathSegment).Reverse()
		newSegs.PushFront(e.Value)
	}
	me.segs = newSegs
}

func (me *Path) Front() PathSegment {
	if me.segs == nil || me.segs.Len() == 0 {
		return nil
	}
	return me.segs.Front().Value.(PathSegment)
}

func (me *Path) FrontPoint() *geom.Coord {
	s := me.Front()
	if s != nil {
		return s.P1()
	}
	return nil
}

func (me *Path) Back() PathSegment {
	if me.segs == nil || me.segs.Len() == 0 {
		return nil
	}
	return me.segs.Back().Value.(PathSegment)
}

func (me *Path) BackPoint() *geom.Coord {
	s := me.Back()
	if s != nil {
		return s.P2()
	}
	return nil
}

func (me *Path) Draw(svg *SVG, s ...string) {
	startP := me.segs.Front().Value.(PathSegment).P1()
	svg.StartPath(*startP, s...)
	for e := me.segs.Front(); e != nil; e = e.Next() {
		e.Value.(PathSegment).PathDraw(svg)
	}
	svg.EndPath()
}

func (me *Path) TotalLength() float64 {
	total := 0.0
	if me.segs == nil {
		return total
	}
	for e := me.segs.Front(); e != nil; e = e.Next() {
		total += e.Value.(PathSegment).Length()
	}
	return total
}

func (me *Path) IsClosed() bool {
	if me.segs == nil || me.segs.Len() == 0 {
		return false
	}

	firstPoint := me.FrontPoint()
	lastPoint := me.BackPoint()

	if firstPoint == nil || lastPoint == nil {
		return false
	}

	// Use a reasonable tolerance for path closure detection
	const closureTolerance = 0.001
	distance := firstPoint.DistanceFrom(*lastPoint)
	return distance < closureTolerance
}

func (me *Path) Area() float64 {
	if !me.IsClosed() {
		return 0
	}
	var points []geom.Coord
	points = append(points, *me.Front().P1())
	for e := me.segs.Front(); e != nil; e = e.Next() {
		points = append(points, *e.Value.(PathSegment).P2())
	}

	area := 0.0
	n := len(points)
	for i := 0; i < n; i++ {
		j := (i + 1) % n
		area += points[i].X * points[j].Y
		area -= points[j].X * points[i].Y
	}
	return math.Abs(area) / 2
}

func getSegments(path *Path) []PathSegment {
	var segments []PathSegment
	for e := path.segs.Front(); e != nil; e = e.Next() {
		segments = append(segments, e.Value.(PathSegment))
	}
	return segments
}

// +++ CutLine
type CutLine struct {
	A, B geom.Coord
}

func AlmostEqualsCutLines(a, b *CutLine) bool {
	return (AlmostEqualsCoord(a.A, b.A) && AlmostEqualsCoord(a.B, b.B)) ||
		(AlmostEqualsCoord(a.A, b.B) && AlmostEqualsCoord(a.B, b.A))
}

func (cl *CutLine) Equals(oi interface{}) bool {
	ocl, ok := oi.(*CutLine)
	return ok && AlmostEqualsCutLines(cl, ocl)
}

func (cl *CutLine) Bounds() geom.Rect {
	r := geom.Rect{cl.A, cl.A}
	r.ExpandToContainCoord(cl.B)
	return r
}

func (cl *CutLine) P1() *geom.Coord { return &cl.A }
func (cl *CutLine) P2() *geom.Coord { return &cl.B }
func (cl *CutLine) PathDraw(svg *SVG) {
	svg.PathLineTo(cl.B)
}
func (cl *CutLine) Reverse() {
	cl.A, cl.B = cl.B, cl.A
}
func (cl *CutLine) Length() float64 {
	return cl.A.DistanceFrom(cl.B)
}

// +++ MarkArc
// This is an arc around point C and radius R, starting at the angle defined
// by point A and ending at angle defined by point B.  The point of the arc
// along C->A will be squished somewhat according to global tuning parameters.
type MarkArc struct {
	C, A, B  geom.Coord
	R        float64
	Offset   float64
	Reversed bool
}

func (ma *MarkArc) P1() *geom.Coord {
	r1 := ma.R*SQUISH_ARC_FACTOR + ma.Offset
	r2 := ma.R + ma.Offset
	if ma.Reversed {
		r1, r2 = r2, r1
	}
	p1 := ma.A.Minus(ma.C).Unit().Times(r1).Plus(ma.C)
	return &p1
}

func (ma *MarkArc) P2() *geom.Coord {
	r1 := ma.R*SQUISH_ARC_FACTOR + ma.Offset
	r2 := ma.R + ma.Offset
	if ma.Reversed {
		r1, r2 = r2, r1
	}
	p2 := ma.B.Minus(ma.C).Unit().Times(r2).Plus(ma.C)
	return &p2
}

func (ma *MarkArc) PathDraw(svg *SVG) {
	if SQUISH_ARC_FUNC == ARC_FUNC_BEZIER {
		ma.PathSquishedArcBezierTo(svg)
	} else {
		ma.PathSquishedArcCircleTo(svg)
	}
}
func (ma *MarkArc) Reverse() {
	ma.A, ma.B = ma.B, ma.A
	ma.Reversed = !ma.Reversed
}

func (ma *MarkArc) PathSquishedArcCircleTo(svg *SVG) {
	r1 := ma.R*SQUISH_ARC_FACTOR + ma.Offset
	r2 := ma.R + ma.Offset
	if ma.Reversed {
		r1, r2 = r2, r1
	}
	p2 := ma.B.Minus(ma.C).Unit().Times(r2).Plus(ma.C)
	a := geom.VertexAngle(ma.A, ma.C, ma.B)
	largeArc := a > math.Pi
	sweep := a > 0
	svg.PathCircularArcTo(p2, ma.R+ma.Offset/2, largeArc, sweep)
}

func (ma *MarkArc) PathSquishedArcBezierTo(svg *SVG, s ...string) {
	r1 := ma.R*SQUISH_ARC_FACTOR + ma.Offset
	r2 := ma.R + ma.Offset
	if ma.Reversed {
		r1, r2 = r2, r1
	}

	v1 := ma.A.Minus(ma.C).Unit()
	v2 := ma.B.Minus(ma.C).Unit()
	p1 := v1.Times(r1).Plus(ma.C)
	p2 := v2.Times(r2).Plus(ma.C)
	a := geom.VertexAngle(ma.A, ma.C, ma.B)

	var v1p, v2p geom.Coord
	if a > 0 {
		v1p = geom.Coord{-v1.Y, v1.X}.Unit()
		v2p = geom.Coord{v2.Y, -v2.X}.Unit()
	} else {
		v1p = geom.Coord{v1.Y, -v1.X}.Unit()
		v2p = geom.Coord{-v2.Y, v2.X}.Unit()
	}
	ctrlDist1 := SQUISH_ARC_BEZIER_ROUNDESS_A * math.Abs(a) * math.Pow(C1, DEFLATE_LEVEL)
	ctrlDist2 := SQUISH_ARC_BEZIER_ROUNDESS_B*math.Abs(a)*math.Pow(C1, DEFLATE_LEVEL) + ma.Offset
	if ma.Reversed {
		ctrlDist1, ctrlDist2 = ctrlDist2, ctrlDist1
	}
	ctrl1 := v1p.Times(ctrlDist1).Plus(p1)
	ctrl2 := v2p.Times(ctrlDist2).Plus(p2)
	// svg.Circle(p1, 0.002, s...)
	// svg.Circle(ctrl1, 0.001, s...)
	svg.PathCubicBezierTo(p2, ctrl1, ctrl2)
}

func (ma *MarkArc) Length() float64 {
	angle := math.Abs(geom.VertexAngle(ma.A, ma.C, ma.B))
	return angle * ma.R
}

// +++ RenderOutput
type RenderOutput struct {
	cuts  []*CutLine
	mark1 []*MarkArc
	mark2 []*MarkArc
}

func (me *RenderOutput) AddCutLine(p1, p2 geom.Coord) {
	me.cuts = append(me.cuts, &CutLine{p1, p2})
}

func (me *RenderOutput) AddMark1Arc(c, v1, v2 geom.Coord, r float64) {
	me.mark1 = append(me.mark1, &MarkArc{c, v1, v2, r, 0.0, false})
}
func (me *RenderOutput) AddMark2Arc(c, v1, v2 geom.Coord, r float64) {
	me.mark2 = append(me.mark2, &MarkArc{c, v1, v2, r, +DOUBLE_STROKE_OFFSET, false})
	me.mark2 = append(me.mark2, &MarkArc{c, v1, v2, r, -DOUBLE_STROKE_OFFSET, false})
}

func (me *RenderOutput) RemoveDuplicates() {
	duplicateStart := time.Now()
	initialCount := len(me.cuts)

	if DEBUG_MODE {
		fmt.Fprintf(os.Stderr, "Number of cut lines before RemoveDuplicates: %d\n", initialCount)
	}

	// Pre-allocate with exact capacity for better performance
	seen := make(map[[4]int64]struct{}, len(me.cuts))
	newCuts := make([]*CutLine, 0, len(me.cuts)/2) // Estimate 50% duplicates

	const precision = 1e10 // Precision factor for float to int conversion

	for _, cl := range me.cuts {
		a, b := cl.A, cl.B
		// Normalize order to ensure consistent key generation
		if a.X > b.X || (a.X == b.X && a.Y > b.Y) {
			a, b = b, a
		}

		// Use array instead of struct for better performance
		key := [4]int64{
			int64(a.X * precision),
			int64(a.Y * precision),
			int64(b.X * precision),
			int64(b.Y * precision),
		}

		if _, exists := seen[key]; !exists {
			seen[key] = struct{}{}
			newCuts = append(newCuts, cl)
		}
	}
	me.cuts = newCuts

	duplicatesRemoved := initialCount - len(me.cuts)
	fmt.Printf("    Removed %d duplicates in %v (%.1f%% reduction)\n",
		duplicatesRemoved, time.Since(duplicateStart),
		float64(duplicatesRemoved)/float64(initialCount)*100)

	if DEBUG_MODE {
		fmt.Fprintf(os.Stderr, "Number of cut lines after RemoveDuplicates: %d\n", len(me.cuts))
	}
}

func (me *RenderOutput) Optimize() *OptimizedRenderOutput {
	or := new(OptimizedRenderOutput)

	// Only process mark1 paths as requested
	for _, ma := range me.mark1 {
		or.AddMark1(ma)
	}

	or.Optimize()

	return or
}

// +++ OptimizedRenderOutput
// BUGBUG: I could refactor this make RenderOutput a special case of this but
// I'm too lazy

type OptimizedPathCollection struct {
	paths []*Path
}

func (opc *OptimizedPathCollection) Draw(svg *SVG, s ...string) {
	for _, path := range opc.paths {
		path.Draw(svg, s...)
	}
}

func (opc *OptimizedPathCollection) NumPaths() int {
	return len(opc.paths)
}

func (opc *OptimizedPathCollection) AddSegment(p PathSegment) {
	path := new(Path)
	path.PushFront(p)
	opc.AddPath(path)
}

func doSegmentsCross(seg1, seg2 PathSegment) bool {
	p1 := seg1.P1()
	p2 := seg1.P2()
	p3 := seg2.P1()
	p4 := seg2.P2()

	// Calculate the orientation of the points
	o1 := orientation(*p1, *p2, *p3)
	o2 := orientation(*p1, *p2, *p4)
	o3 := orientation(*p3, *p4, *p1)
	o4 := orientation(*p3, *p4, *p2)

	// General case
	if o1 != o2 && o3 != o4 {
		return true
	}

	// Special cases
	if o1 == 0 && onSegment(*p1, *p3, *p2) {
		return true
	}
	if o2 == 0 && onSegment(*p1, *p4, *p2) {
		return true
	}
	if o3 == 0 && onSegment(*p3, *p1, *p4) {
		return true
	}
	if o4 == 0 && onSegment(*p3, *p2, *p4) {
		return true
	}

	return false
}

func (opc *OptimizedPathCollection) AddPath(np *Path) {
	npP1 := np.Front().P1()
	npP2 := np.Back().P2()
	for _, path := range opc.paths {
		if AlmostEqualsCoord(*npP2, *path.Front().P1()) {
			path.PushPathFront(np)
			return
		}
		if AlmostEqualsCoord(*npP1, *path.Back().P2()) {
			path.PushPathBack(np)
			return
		}
		if AlmostEqualsCoord(*npP1, *path.Front().P1()) {
			np.Reverse()
			path.PushPathFront(np)
			return
		}
		if AlmostEqualsCoord(*npP2, *path.Back().P2()) {
			np.Reverse()
			path.PushPathBack(np)
			return
		}
	}

	opc.paths = append(opc.paths, np)
}

func (opc *OptimizedPathCollection) Optimize() {
	// Use the optimized connected paths algorithm by default
	opc.OptimizeConnectedPaths()
}

func (opc *OptimizedPathCollection) OptimizeWithTravel(enableTravelOpt bool) {
	// Use the same optimization but with travel optimization controlled by parameter
	// Replace the current content with optimized version
	opc.OptimizeConnectedPaths()

	// Only do travel optimization if enabled
	if enableTravelOpt && len(opc.paths) > 1 {
		opc.OptimizeTravelDistance()
	}
}

func (opc *OptimizedPathCollection) OptimizeConnectedPaths() {
	if DEBUG_MODE {
		fmt.Fprintf(os.Stderr, "  Number of paths before optimization: %d\n", len(opc.paths))
	}

	if ENABLE_FAST_PATH_OPTIMIZATION && len(opc.paths) > 100 {
		opc.OptimizeConnectedPathsFast()
	} else {
		opc.OptimizeConnectedPathsSlow()
	}

	if DEBUG_MODE {
		fmt.Fprintf(os.Stderr, "  Number of paths after optimization: %d\n", len(opc.paths))
	}
}

// Simple path optimization - connect paths with reasonable endpoint tolerance
func (opc *OptimizedPathCollection) OptimizeConnectedPathsSimple() {
	if DEBUG_MODE {
		fmt.Fprintf(os.Stderr, "  Number of paths before simple optimization: %d\n", len(opc.paths))
	}

	// More aggressive but controlled optimization - connect arcs that should form closed paths
	changed := true
	maxIterations := 10 // Increased iterations to allow more connections
	iteration := 0

	for changed && iteration < maxIterations {
		changed = false
		iteration++

		for i := len(opc.paths) - 1; i >= 0; i-- {
			if i >= len(opc.paths) {
				continue // Path was removed in previous iteration
			}

			pathI := opc.paths[i]
			pathIBack := pathI.BackPoint()
			pathIFront := pathI.FrontPoint()

			for j := len(opc.paths) - 1; j >= 0; j-- {
				if i == j || j >= len(opc.paths) {
					continue
				}

				pathJ := opc.paths[j]
				pathJFront := pathJ.FrontPoint()
				pathJBack := pathJ.BackPoint()

				// Use reasonable tolerance for Penrose arc connections
				const connectionTolerance = 0.05 // Further relaxed for better connections

				// Can we connect the back of pathI to the front of pathJ?
				if pathIBack != nil && pathJFront != nil {
					dist := pathIBack.DistanceFrom(*pathJFront)
					if dist < connectionTolerance {
						pathI.PushPathBack(pathJ)
						opc.paths = append(opc.paths[:j], opc.paths[j+1:]...)
						changed = true
						break
					}
				}

				// Can we connect the front of pathI to the back of pathJ?
				if pathIFront != nil && pathJBack != nil {
					dist := pathIFront.DistanceFrom(*pathJBack)
					if dist < connectionTolerance {
						pathJ.PushPathBack(pathI)
						opc.paths = append(opc.paths[:i], opc.paths[i+1:]...)
						changed = true
						break
					}
				}

				// Can we connect the back of pathI to the back of pathJ? (reverse pathJ)
				if pathIBack != nil && pathJBack != nil {
					dist := pathIBack.DistanceFrom(*pathJBack)
					if dist < connectionTolerance {
						pathJ.Reverse()
						pathI.PushPathBack(pathJ)
						opc.paths = append(opc.paths[:j], opc.paths[j+1:]...)
						changed = true
						break
					}
				}

				// Can we connect the front of pathI to the front of pathJ? (reverse pathI)
				if pathIFront != nil && pathJFront != nil {
					dist := pathIFront.DistanceFrom(*pathJFront)
					if dist < connectionTolerance {
						pathI.Reverse()
						pathI.PushPathBack(pathJ)
						opc.paths = append(opc.paths[:j], opc.paths[j+1:]...)
						changed = true
						break
					}
				}
			}
			if changed {
				break
			}
		}
	}

	if DEBUG_MODE {
		fmt.Fprintf(os.Stderr, "  Number of paths after simple optimization: %d (iterations: %d)\n", len(opc.paths), iteration)
	}
}
func (opc *OptimizedPathCollection) OptimizeConnectedPathsFast() {
	if len(opc.paths) == 0 {
		return
	}

	// Create spatial hash map for endpoints
	type endpoint struct {
		pathIdx int
		isBack  bool
		coord   geom.Coord
	}

	// Hash function for coordinates (quantize to grid)
	const gridSize = 0.0001
	hashCoord := func(c geom.Coord) [2]int64 {
		return [2]int64{
			int64(c.X / gridSize),
			int64(c.Y / gridSize),
		}
	}

	endpointMap := make(map[[2]int64][]endpoint)

	// Build spatial hash map
	for i, path := range opc.paths {
		if frontPoint := path.FrontPoint(); frontPoint != nil {
			hash := hashCoord(*frontPoint)
			endpointMap[hash] = append(endpointMap[hash], endpoint{i, false, *frontPoint})
		}
		if backPoint := path.BackPoint(); backPoint != nil {
			hash := hashCoord(*backPoint)
			endpointMap[hash] = append(endpointMap[hash], endpoint{i, true, *backPoint})
		}
	}

	// Track merged paths
	merged := make([]bool, len(opc.paths))
	changed := true

	for changed && len(opc.paths) > 1 {
		changed = false

		for i := 0; i < len(opc.paths) && !changed; i++ {
			if merged[i] {
				continue
			}

			pathI := opc.paths[i]
			pathIBack := pathI.BackPoint()
			pathIFront := pathI.FrontPoint()

			// Check connections using spatial hash
			if pathIBack != nil {
				hash := hashCoord(*pathIBack)
				for _, candidates := range [][]endpoint{endpointMap[hash]} {
					for _, ep := range candidates {
						if merged[ep.pathIdx] || ep.pathIdx == i {
							continue
						}

						if !ep.isBack && AlmostEqualsCoord(*pathIBack, ep.coord) {
							// Connect back of pathI to front of pathJ
							pathJ := opc.paths[ep.pathIdx]
							pathI.PushPathBack(pathJ)
							merged[ep.pathIdx] = true
							changed = true
							break
						}
					}
					if changed {
						break
					}
				}
			}

			if !changed && pathIFront != nil {
				hash := hashCoord(*pathIFront)
				for _, candidates := range [][]endpoint{endpointMap[hash]} {
					for _, ep := range candidates {
						if merged[ep.pathIdx] || ep.pathIdx == i {
							continue
						}

						if ep.isBack && AlmostEqualsCoord(*pathIFront, ep.coord) {
							// Connect front of pathI to back of pathJ
							pathJ := opc.paths[ep.pathIdx]
							pathJ.PushPathBack(pathI)
							merged[i] = true
							changed = true
							break
						}
					}
					if changed {
						break
					}
				}
			}
		}

		if changed {
			// Rebuild paths array without merged paths
			newPaths := make([]*Path, 0, len(opc.paths))
			for i, path := range opc.paths {
				if !merged[i] {
					newPaths = append(newPaths, path)
				}
			}
			opc.paths = newPaths

			// Rebuild spatial hash map
			endpointMap = make(map[[2]int64][]endpoint)
			merged = make([]bool, len(opc.paths))

			for i, path := range opc.paths {
				if frontPoint := path.FrontPoint(); frontPoint != nil {
					hash := hashCoord(*frontPoint)
					endpointMap[hash] = append(endpointMap[hash], endpoint{i, false, *frontPoint})
				}
				if backPoint := path.BackPoint(); backPoint != nil {
					hash := hashCoord(*backPoint)
					endpointMap[hash] = append(endpointMap[hash], endpoint{i, true, *backPoint})
				}
			}
		}
	}
}

// Original O(n²) path optimization - kept for small datasets or as fallback
func (opc *OptimizedPathCollection) OptimizeConnectedPathsSlow() {
	// This is a slow operation.  It tries to connect up paths that share endpoints
	// so that a path cutting tool doesn't have to do as much backtracking.
	changed := true
	for changed {
		changed = false
		for i, pathI := range opc.paths {
			pathIBack := pathI.BackPoint()
			pathIFront := pathI.FrontPoint()
			for j, pathJ := range opc.paths {
				if i == j {
					continue
				}
				pathJFront := pathJ.FrontPoint()
				pathJBack := pathJ.BackPoint()

				// Can we connect the back of pathI to the front of pathJ?
				if pathIBack != nil && pathJFront != nil && AlmostEqualsCoord(*pathIBack, *pathJFront) {
					pathI.PushPathBack(pathJ)
					opc.paths = append(opc.paths[:j], opc.paths[j+1:]...)
					changed = true
					break
				}

				// Can we connect the front of pathI to the back of pathJ?
				if pathIFront != nil && pathJBack != nil && AlmostEqualsCoord(*pathIFront, *pathJBack) {
					pathJ.PushPathBack(pathI)
					opc.paths = append(opc.paths[:i], opc.paths[i+1:]...)
					changed = true
					break
				}

				// Can we connect the back of pathI to the back of pathJ?
				if pathIBack != nil && pathJBack != nil && AlmostEqualsCoord(*pathIBack, *pathJBack) {
					pathJ.Reverse()
					pathI.PushPathBack(pathJ)
					opc.paths = append(opc.paths[:j], opc.paths[j+1:]...)
					changed = true
					break
				}

				// Can we connect the front of pathI to the front of pathJ?
				if pathIFront != nil && pathJFront != nil && AlmostEqualsCoord(*pathIFront, *pathJFront) {
					pathI.Reverse()
					pathI.PushPathBack(pathJ)
					opc.paths = append(opc.paths[:j], opc.paths[j+1:]...)
					changed = true
					break
				}
			}
			if changed {
				break
			}
		}
	}
}

func (opc *OptimizedPathCollection) OptimizeTravelDistance() {
	if len(opc.paths) <= 1 {
		return
	}

	// Fast nearest-neighbor travel optimization
	optimized := make([]*Path, 0, len(opc.paths))
	used := make(map[int]bool)

	// Start with first path
	optimized = append(optimized, opc.paths[0])
	used[0] = true

	// Greedily select nearest unused path (limited search for performance)
	for len(optimized) < len(opc.paths) {
		currentBack := optimized[len(optimized)-1].BackPoint()
		if currentBack == nil {
			// Find any unused path
			for i := range opc.paths {
				if !used[i] {
					optimized = append(optimized, opc.paths[i])
					used[i] = true
					break
				}
			}
			continue
		}

		bestDist := math.MaxFloat64
		bestIdx := -1

		// Limit search to first 50 paths for performance
		checked := 0
		for i := range opc.paths {
			if used[i] || checked > 50 {
				continue
			}
			checked++

			path := opc.paths[i]
			if frontPoint := path.FrontPoint(); frontPoint != nil {
				dist := currentBack.DistanceFrom(*frontPoint)
				if dist < bestDist {
					bestDist = dist
					bestIdx = i
				}
			}
		}

		if bestIdx >= 0 {
			optimized = append(optimized, opc.paths[bestIdx])
			used[bestIdx] = true
		} else {
			// Fallback: find any unused path
			for i := range opc.paths {
				if !used[i] {
					optimized = append(optimized, opc.paths[i])
					used[i] = true
					break
				}
			}
		}
	}

	opc.paths = optimized
}

func (opc *OptimizedPathCollection) PrintPathLengths() {
	total := 0.0
	for i, path := range opc.paths {
		length := path.TotalLength()
		total += length
		fmt.Printf("Path %d length: %.4f\n", i+1, length)
	}
	fmt.Printf("Total length of all paths: %.4f\n", total)
}

func (opc *OptimizedPathCollection) Filtered(maxLength float64) *OptimizedPathCollection {
	filtered := &OptimizedPathCollection{}
	for _, path := range opc.paths {
		if path.TotalLength() <= maxLength {
			filtered.paths = append(filtered.paths, path)
		}
	}
	return filtered
}

func (opc *OptimizedPathCollection) RemoveUnclosedPaths() {
	var validPaths []*Path
	closedCount := 0
	unclosedCount := 0

	for i, path := range opc.paths {
		isClosed := path.IsClosed()
		if isClosed {
			validPaths = append(validPaths, path)
			closedCount++
		} else {
			unclosedCount++
			// Debug first few unclosed paths
			if i < 3 {
				frontPoint := path.FrontPoint()
				backPoint := path.BackPoint()
				if frontPoint != nil && backPoint != nil {
					dist := frontPoint.DistanceFrom(*backPoint)
					fmt.Printf("      Debug: Path %d unclosed, front-back distance: %.6f\n", i+1, dist)
				}
			}
		}
	}

	fmt.Printf("    Debug: %d closed, %d unclosed paths\n", closedCount, unclosedCount)
	opc.paths = validPaths
}

func (opc *OptimizedPathCollection) CheckCrossings() {
	for i, path1 := range opc.paths {
		for j, path2 := range opc.paths {
			if i == j {
				continue
			}
			for _, seg1 := range getSegments(path1) {
				for _, seg2 := range getSegments(path2) {
					if doSegmentsCross(seg1, seg2) {
						fmt.Printf("Crossing detected between path %d and path %d\n", i+1, j+1)
						return
					}
				}
			}
		}
	}
	fmt.Println("No crossings detected.")
}

// Group analysis and filtering
type PathGroup struct {
	paths       []*Path
	avgLength   float64
	totalLength float64
}

func (opc *OptimizedPathCollection) AnalyzeGroups() map[float64]PathGroup {
	if len(opc.paths) == 0 {
		return make(map[float64]PathGroup)
	}

	// Group paths by length ranges - increased tolerance for better grouping
	groups := make(map[float64]PathGroup)
	const lengthTolerance = 0.01 // Increased from 0.001 to 0.01

	// Collect all lengths for debugging
	var allLengths []float64
	for _, path := range opc.paths {
		allLengths = append(allLengths, path.TotalLength())
	}
	sort.Float64s(allLengths)

	fmt.Printf("    Debug: Path lengths range from %.6f to %.6f\n",
		allLengths[0], allLengths[len(allLengths)-1])

	for _, path := range opc.paths {
		length := path.TotalLength()

		// Find existing group with similar length
		var groupKey float64 = -1
		for key := range groups {
			if math.Abs(length-key) < lengthTolerance {
				groupKey = key
				break
			}
		}

		if groupKey == -1 {
			// Create new group
			groupKey = length
			groups[groupKey] = PathGroup{
				paths:       []*Path{path},
				avgLength:   length,
				totalLength: length,
			}
		} else {
			// Add to existing group
			group := groups[groupKey]
			group.paths = append(group.paths, path)
			group.totalLength += length
			group.avgLength = group.totalLength / float64(len(group.paths))
			groups[groupKey] = group
		}
	}

	return groups
}

func (opc *OptimizedPathCollection) KeepTwoSmallestGroups() {
	groups := opc.AnalyzeGroups()

	fmt.Printf("    Found %d groups from %d paths\n", len(groups), len(opc.paths))

	// Sort groups by average length
	type groupInfo struct {
		key   float64
		group PathGroup
	}

	// Show all groups for debugging
	var sortedGroupsDebug []groupInfo
	for key, group := range groups {
		sortedGroupsDebug = append(sortedGroupsDebug, groupInfo{key, group})
	}
	sort.Slice(sortedGroupsDebug, func(i, j int) bool {
		return sortedGroupsDebug[i].group.avgLength < sortedGroupsDebug[j].group.avgLength
	})

	for i, g := range sortedGroupsDebug {
		fmt.Printf("      Group %d: %d paths, avg length %.6f\n",
			i+1, len(g.group.paths), g.group.avgLength)
	}

	if len(groups) <= 2 {
		fmt.Printf("    Only %d groups found, keeping all paths\n", len(groups))
		return // Already 2 or fewer groups
	}

	var sortedGroups []groupInfo
	for key, group := range groups {
		sortedGroups = append(sortedGroups, groupInfo{key, group})
	}

	sort.Slice(sortedGroups, func(i, j int) bool {
		return sortedGroups[i].group.avgLength < sortedGroups[j].group.avgLength
	})

	// Keep only the two smallest groups
	var newPaths []*Path
	for i := 0; i < 2 && i < len(sortedGroups); i++ {
		newPaths = append(newPaths, sortedGroups[i].group.paths...)
		fmt.Printf("    ✓ Keeping group %d: %d paths, avg length %.6f\n",
			i+1, len(sortedGroups[i].group.paths), sortedGroups[i].group.avgLength)
	}

	fmt.Printf("    Reduced from %d paths in %d groups to %d paths in 2 groups\n",
		len(opc.paths), len(groups), len(newPaths))

	opc.paths = newPaths
}

type OptimizedRenderOutput struct {
	cuts  OptimizedPathCollection
	mark1 OptimizedPathCollection
	mark2 OptimizedPathCollection
}

func (me *OptimizedRenderOutput) Optimize() {
	mark1Start := time.Now()
	if DEBUG_MODE {
		fmt.Fprintf(os.Stderr, "Optimizing mark1 paths\n")
	}

	// Simple optimization - use the regular slow optimization which works better for exact connections
	me.mark1.OptimizeConnectedPathsSlow()

	if OPTIMIZE_MARK_TRAVEL {
		me.mark1.OptimizeWithTravel(true)
	}
	fmt.Printf("    ✓ Mark1 paths optimized: %v (%d paths)\n", time.Since(mark1Start), me.mark1.NumPaths())

	// Remove unclosed paths to get only closed figures
	unclosedStart := time.Now()
	initialPaths := me.mark1.NumPaths()
	me.mark1.RemoveUnclosedPaths()
	fmt.Printf("    ✓ Removed unclosed paths: %v (%d → %d paths, %.1f%% reduction)\n",
		time.Since(unclosedStart), initialPaths, me.mark1.NumPaths(),
		float64(initialPaths-me.mark1.NumPaths())/float64(initialPaths)*100)

	// Analyze and filter to keep only two smallest groups
	groupStart := time.Now()
	groupInitialPaths := me.mark1.NumPaths()
	groups := me.mark1.AnalyzeGroups()
	fmt.Printf("    ✓ Analyzed path groups: %v (%d groups found from %d paths)\n",
		time.Since(groupStart), len(groups), groupInitialPaths)

	// Keep only the two smallest groups as requested
	me.mark1.KeepTwoSmallestGroups()
	fmt.Printf("    ✓ Filtered to two smallest groups: (%d → %d paths)\n",
		groupInitialPaths, me.mark1.NumPaths())
}

func (me *OptimizedRenderOutput) AddCut(p PathSegment) {
	me.cuts.AddSegment(p)
}

func (me *OptimizedRenderOutput) AddMark1(p PathSegment) {
	me.mark1.AddSegment(p)
}

func (me *OptimizedRenderOutput) AddMark2(p PathSegment) {
	me.mark2.AddSegment(p)
}

func (opc *OptimizedPathCollection) ScaleToGoldenRatio(mode ScaleMode) float64 {
	lightArea := 0.0
	for _, path := range opc.paths {
		if path.IsClosed() {
			lightArea += path.Area()
		}
	}

	decagonArea := DecagonArea(1.0)
	darkArea := decagonArea - lightArea
	psi := lightArea / decagonArea

	a := 0.0 // a = scaleFactor^2
	switch mode {
	case LightToDark:
		a = 1 / (psi * (math.Phi + 1))
	case DarkToLight:
		a = 1 / (psi * (1/math.Phi + 1))
	case LightToTotal:
		a = 1 / (math.Phi * psi)
	case DarkToTotal:
		a = (1 - 1/math.Phi) / psi
	}

	scaleFactor := math.Sqrt(a)
	fmt.Printf("Scale factor calculated: %.6f\n", scaleFactor)

	// Scale only the closed paths (light areas) relative to their own center
	for _, path := range opc.paths {
		if !path.IsClosed() {
			continue
		}

		var sumX, sumY float64
		pointCount := 0
		points := make(map[geom.Coord]bool)
		points[*path.Front().P1()] = true
		for e := path.segs.Front(); e != nil; e = e.Next() {
			points[*e.Value.(PathSegment).P2()] = true
		}

		for point := range points {
			sumX += point.X
			sumY += point.Y
			pointCount++
		}

		center := geom.Coord{X: sumX / float64(pointCount), Y: sumY / float64(pointCount)}

		// Apply scaling to this path's segments
		for e := path.segs.Front(); e != nil; e = e.Next() {
			switch seg := e.Value.(type) {
			case *CutLine:
				seg.A = center.Plus(seg.A.Minus(center).Times(scaleFactor))
				seg.B = center.Plus(seg.B.Minus(center).Times(scaleFactor))
			case *MarkArc:
				seg.A = center.Plus(seg.A.Minus(center).Times(scaleFactor))
				seg.B = center.Plus(seg.B.Minus(center).Times(scaleFactor))
				seg.C = center.Plus(seg.C.Minus(center).Times(scaleFactor))
				seg.R *= scaleFactor
			}
		}
	}

	// Recalculate areas after scaling only the light paths
	lightArea = 0.0
	for _, path := range opc.paths {
		if path.IsClosed() {
			lightArea += path.Area()
		}
	}

	darkArea = decagonArea - lightArea
	currentRatio := 0.0
	switch mode {
	case LightToDark:
		currentRatio = lightArea / darkArea
	case DarkToLight:
		currentRatio = darkArea / lightArea
	case LightToTotal:
		currentRatio = lightArea / decagonArea
	case DarkToTotal:
		currentRatio = darkArea / decagonArea
	}

	fmt.Printf("Achieved ratio: %.6f\n", currentRatio)
	return scaleFactor
}

// Generate multiple scaled output files
func GenerateScaledImages(oro *OptimizedRenderOutput, bounds geom.Rect) {
	os.MkdirAll("output", 0755)

	// First, we need to analyze the paths and separate them into two groups
	groups := oro.mark1.AnalyzeGroups()
	if len(groups) < 2 {
		fmt.Printf("Warning: Only %d groups found, expected at least 2\n", len(groups))
		return
	}

	// Sort groups by average length to get the two smallest groups
	type groupInfo struct {
		key   float64
		group PathGroup
	}

	var sortedGroups []groupInfo
	for key, group := range groups {
		sortedGroups = append(sortedGroups, groupInfo{key, group})
	}

	sort.Slice(sortedGroups, func(i, j int) bool {
		return sortedGroups[i].group.avgLength < sortedGroups[j].group.avgLength
	})

	// Generate files for the two smallest groups only
	scaleModes := []ScaleMode{DarkToLight, DarkToTotal} // dtl = bright, dtt = dark
	fileNames := []string{"bright", "dark"}

	// Process each of the two smallest groups
	for groupIdx := 0; groupIdx < 2 && groupIdx < len(sortedGroups); groupIdx++ {
		groupName := fmt.Sprintf("group%d", groupIdx+1)
		groupPaths := sortedGroups[groupIdx].group.paths

		fmt.Printf("    Processing %s: %d paths, avg length %.6f\n",
			groupName, len(groupPaths), sortedGroups[groupIdx].group.avgLength)

		// Create collection for this specific group
		groupCollection := OptimizedPathCollection{
			paths: make([]*Path, len(groupPaths)),
		}
		copy(groupCollection.paths, groupPaths)

		// Generate files for each scale mode
		for modeIdx, mode := range scaleModes {
			fileName := fileNames[modeIdx]

			// Create a copy for scaling
			scaledCollection := OptimizedPathCollection{
				paths: make([]*Path, len(groupCollection.paths)),
			}
			copy(scaledCollection.paths, groupCollection.paths)

			// Scale the collection
			scaledCollection.ScaleToGoldenRatio(mode)

			// Generate the SVG file with black background and white stroke
			file, err := os.Create(fmt.Sprintf("output/%s_%s.svg", groupName, fileName))
			if err != nil {
				fmt.Printf("Error creating %s_%s file: %v\n", groupName, fileName, err)
				continue
			}

			s := NewSVG(file)
			// Black background with white stroke for visibility - combine styles properly
			combinedStyle := "background-color: black; " + DEFAULT_STYLE
			s.Start(bounds, "style='"+combinedStyle+"'")
			scaledCollection.Draw(s, "stroke: white; fill: none")
			s.End()
			file.Close()

			fmt.Printf("    ✓ Generated %s_%s.svg (mode: %s, %d paths)\n",
				groupName, fileName, string(mode), scaledCollection.NumPaths())
		}
	}
}

// //////////////////////////////////////////////////////////////////////////
// Primary Penrose tile generation
type PenrosePrimitive interface {
	Render(ro *RenderOutput)
	Deflate() []PenrosePrimitive
}

// +++ halfKite
type halfKite struct {
	*geom.Triangle
}

func (me halfKite) Render(ro *RenderOutput) {
	ro.AddCutLine(me.B, me.C)
	ro.AddCutLine(me.C, me.A)

	rB := me.A.Minus(me.C).Magnitude()
	rA := rB * 1.0 / math.Phi

	ro.AddMark2Arc(me.A, me.B, me.C, rA)
	ro.AddMark1Arc(me.B, me.A, me.C, rB)
}

func (me halfKite) Deflate() []PenrosePrimitive {
	r := make([]PenrosePrimitive, 3)
	d := me.A.Times(C1).Plus(me.B.Times(C2))
	e := me.B.Times(C1).Plus(me.C.Times(C2))
	r[0] = halfKite{&geom.Triangle{d, me.C, me.A}}
	r[1] = halfKite{&geom.Triangle{d, me.C, e}}
	r[2] = halfDart{&geom.Triangle{me.B, e, d}}
	return r
}

// +++ halfDart
type halfDart struct {
	*geom.Triangle
}

func (me halfDart) Render(ro *RenderOutput) {
	ro.AddCutLine(me.B, me.C)
	ro.AddCutLine(me.C, me.A)

	r := me.A.Minus(me.C).Magnitude()
	rA := r / math.Pow(math.Phi, 2)
	rB := r / math.Pow(math.Phi, 3)

	ro.AddMark1Arc(me.A, me.B, me.C, rA)
	ro.AddMark2Arc(me.B, me.A, me.C, rB)
}

func (me halfDart) Deflate() []PenrosePrimitive {
	r := make([]PenrosePrimitive, 2)
	d := me.A.Times(C2).Plus(me.C.Times(C1))
	r[0] = halfDart{&geom.Triangle{me.C, d, me.B}}
	r[1] = halfKite{&geom.Triangle{me.B, me.A, d}}
	return r
}

// +++ Starting Shapes
func degToRads(d float64) float64 {
	return d * math.Pi / 180.0
}

var HalfKite = halfKite{
	&geom.Triangle{
		geom.Coord{0, 0},
		geom.Coord{math.Phi * math.Cos(degToRads(72)), math.Phi * math.Sin(degToRads(72))},
		geom.Coord{1, 0},
	},
}

var HalfDart = halfDart{
	&geom.Triangle{
		geom.Coord{1, 0},
		geom.Coord{C1 * math.Cos(degToRads(36)), C1 * math.Sin(degToRads(36))},
		geom.Coord{0, 0},
	},
}

type ScaleMode string

const (
	LightToDark  ScaleMode = "ltd" // light/dark = 1/phi
	DarkToLight  ScaleMode = "dtl" // dark/light = 1/phi
	LightToTotal ScaleMode = "ltt" // light/(light+dark)
	DarkToTotal  ScaleMode = "dtt" // dark/(light+dark) = 1/phi
)

func Sun() []PenrosePrimitive {
	r := make([]PenrosePrimitive, 0, 50) // was 10
	for i := 0; i < 5; i++ {
		r = append(r, halfKite{
			&geom.Triangle{
				geom.Coord{math.Cos(degToRads(float64(72 * i))), math.Sin(degToRads(float64(72 * i)))},
				geom.Coord{0, 0},
				geom.Coord{math.Cos(degToRads(float64(36 + 72.0*i))), math.Sin(degToRads(float64(36 + 72.0*i)))},
			},
		})
		r = append(r, halfKite{
			&geom.Triangle{
				geom.Coord{math.Cos(degToRads(float64(72 * i))), math.Sin(degToRads(float64(72 * i)))},
				geom.Coord{0, 0},
				geom.Coord{math.Cos(degToRads(float64(-36 + 72.0*i))), math.Sin(degToRads(float64(-36 + 72.0*i)))},
			},
		})
	}
	return r
}

func DeflatePenrosePrimitives(ps []PenrosePrimitive, levels int) []PenrosePrimitive {
	r := ps
	if DEBUG_MODE {
		fmt.Fprintf(os.Stderr, "Starting primitive count: %d\n", len(r))
	}

	for i := 0; i < levels; i++ {
		levelStart := time.Now()
		rNext := make([]PenrosePrimitive, 0)
		for _, shape := range r {
			rNext = append(rNext, shape.Deflate()...)
		}
		r = rNext
		levelTime := time.Since(levelStart)
		if DEBUG_MODE {
			fmt.Fprintf(os.Stderr, "Level %d complete: %d primitives in %v\n", i+1, len(r), levelTime)
		} else {
			fmt.Printf("  Deflation level %d/%d: %d primitives (%v)\n", i+1, levels, len(r), levelTime)
		}
	}
	return r
}

func CullShapes(ps []PenrosePrimitive, bounds *geom.Rect) []PenrosePrimitive {
	r := make([]PenrosePrimitive, 0, len(ps))
	for _, shape := range ps {
		if bounds.ContainsRect(shape.(geom.Bounded).Bounds()) {
			r = append(r, shape)
		}
	}
	if DEBUG_MODE {
		fmt.Fprintf(os.Stderr, "Primitive count after cull: %d\n", len(r))
	}
	return r
}

func BoundsOfPrimitiveSlice(ps []PenrosePrimitive) geom.Rect {
	bounds := ps[0].(geom.Bounded).Bounds()
	for _, p := range ps[1:] {
		bounds.ExpandToContainRect(p.(geom.Bounded).Bounds())
	}
	return bounds
}

func CreateCenteredDecagon(radius float64) []geom.Coord {
	points := make([]geom.Coord, 10)
	decagonAngle := math.Pi / 5
	for i := 0; i < 10; i++ {
		angle := float64(i) * decagonAngle // - math.Pi / 2
		points[i] = geom.Coord{
			X: radius * math.Cos(angle),
			Y: radius * math.Sin(angle),
		}
	}
	return points
}

func DecagonArea(radius float64) float64 {
	return 5.0 / 2.0 * radius * radius * math.Sin(math.Pi/5)
}

func orientation(p, q, r geom.Coord) int {
	val := (q.Y-p.Y)*(r.X-q.X) - (q.X-p.X)*(r.Y-q.Y)
	if val == 0 {
		return 0 // colinear
	}
	if val > 0 {
		return 1 // clock wise
	}
	return 2 // counterclock wise
}

func onSegment(p, q, r geom.Coord) bool {
	if q.X <= math.Max(p.X, r.X) && q.X >= math.Min(p.X, r.X) &&
		q.Y <= math.Max(p.Y, r.Y) && q.Y >= math.Min(p.Y, r.Y) {
		return true
	}
	return false
}

func main() {
	totalStart := time.Now()
	fmt.Printf("Starting Penrose circle generation at %s\n", totalStart.Format("15:04:05"))

	// Step 1: Initialize shapes
	stepStart := time.Now()
	shapes := Sun()
	bounds := geom.Rect{geom.Coord{-300, -200}, geom.Coord{300, 200}}
	bounds.Scale(1.0/150.0, 1.0/150.0)
	fmt.Printf("✓ Step 1 - Initialize shapes: %v (%d initial shapes)\n", time.Since(stepStart), len(shapes))

	// Step 2: Deflate shapes
	stepStart = time.Now()
	shapes = DeflatePenrosePrimitives(shapes, DEFLATE_LEVEL)
	fmt.Printf("✓ Step 2 - Deflate primitives (level %d): %v (%d shapes after deflation)\n",
		DEFLATE_LEVEL, time.Since(stepStart), len(shapes))

	// Step 3: Cull shapes
	stepStart = time.Now()
	shapes = CullShapes(shapes, &bounds)
	fmt.Printf("✓ Step 3 - Cull shapes: %v (%d shapes after culling)\n", time.Since(stepStart), len(shapes))

	// Step 4: Render shapes
	stepStart = time.Now()
	ro := new(RenderOutput)
	for _, shape := range shapes {
		shape.Render(ro)
	}
	fmt.Printf("✓ Step 4 - Render shapes: %v (cuts: %d, mark1: %d, mark2: %d)\n",
		time.Since(stepStart), len(ro.cuts), len(ro.mark1), len(ro.mark2))

	// Step 5: Remove duplicates
	stepStart = time.Now()
	ro.RemoveDuplicates()
	fmt.Printf("✓ Step 5 - Remove duplicates: %v (%d cuts remaining)\n", time.Since(stepStart), len(ro.cuts))

	// Step 6: Optimize paths (mark1 only, remove unclosed, keep two smallest groups)
	stepStart = time.Now()
	oro := ro.Optimize()
	fmt.Printf("✓ Step 6 - Optimize mark1 paths: %v (%d paths)\n",
		time.Since(stepStart), oro.mark1.NumPaths())

	// Step 7: Generate multiple output files
	stepStart = time.Now()
	GenerateScaledImages(oro, bounds)
	fmt.Printf("✓ Step 7 - Generate scaled output files: %v\n", time.Since(stepStart))

	totalTime := time.Since(totalStart)
	fmt.Printf("\n🎉 Total generation time: %v\n", totalTime)
	fmt.Printf("Finished at %s\n", time.Now().Format("15:04:05"))
}
