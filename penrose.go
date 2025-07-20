package main

import (
	"container/list"
	"fmt"
	"github.com/jbeda/geom"
	"io"
	"math"
	"os"
	"strings"
	"sync"
	"sort"
)

// Arc drawing functions/strategies
type ArcFuncType int

const (
	ARC_FUNC_CIRCULAR ArcFuncType = iota
	ARC_FUNC_BEZIER
)

// Tunable constants for output
const (
	DEFAULT_STYLE                = "stroke-width: 0.002; stroke-linecap: round; fill: none"
	DOUBLE_STROKE_OFFSET         = 0.002
	CUT_STYLE                    = "stroke: black"
	MARK1_STYLE                  = "stroke: green"
	MARK2_STYLE                  = "stroke: red"
	DEFLATE_LEVEL                = 8
	SQUISH_ARC_FUNC              = ARC_FUNC_CIRCULAR
	SQUISH_ARC_FACTOR            = 0.9
	SQUISH_ARC_BEZIER_ROUNDESS_A = 0.15
	SQUISH_ARC_BEZIER_ROUNDESS_B = 0.1
)

// Mathematical constants for generating Penrose decompositions.
const (
	C1 = math.Phi - 1.0
	C2 = 2.0 - math.Phi
)

////////////////////////////////////////////////////////////////////////////
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

	return firstPoint != nil && lastPoint != nil && AlmostEqualsCoord(*firstPoint, *lastPoint)
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
		j := (i+1) % n
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
	fmt.Fprintf(os.Stderr, "Number of cut lines before RemoveDuplicates: %d\n", len(me.cuts))
	newCuts := []*CutLine{}
OuterLoop:
	for _, cl := range me.cuts {
		for _, ncl := range newCuts {
			if AlmostEqualsCutLines(cl, ncl) {
				continue OuterLoop
			}
		}
		// Didn't find it, add it
		newCuts = append(newCuts, cl)
	}
	me.cuts = newCuts
	fmt.Fprintf(os.Stderr, "Number of cut lines after RemoveDuplicates: %d\n", len(me.cuts))
}

func (me *RenderOutput) Optimize() *OptimizedRenderOutput {
	or := new(OptimizedRenderOutput)
	for _, ma := range me.mark1 {
		or.AddMark1(ma)
	}
	for _, ma := range me.mark2 {
		or.AddMark2(ma)
	}
	for _, c := range me.cuts {
		or.AddCut(c)
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
	fmt.Fprintf(os.Stderr, "  Number of paths before optimization: %d\n", len(opc.paths))
	// Loop through until the number of paths stabilizes
	for i := 0; ; i++ {
		prevNumPaths := len(opc.paths)

		oldPaths := opc.paths
		opc.paths = nil
		for _, p := range oldPaths {
			opc.AddPath(p)
		}

		fmt.Fprintf(os.Stderr, "  Number of paths after interation %d: %d\n", i, len(opc.paths))
		if prevNumPaths == len(opc.paths) {
			break
		}
	}

	// Now sort the paths to minimize non-cutting distance.

	// Compute the non-cutting distance before
	travelDistance := float64(0)
	lastPoint := opc.paths[0].BackPoint()
	for _, p := range opc.paths[1:] {
		travelDistance += lastPoint.DistanceFrom(*p.FrontPoint())
		lastPoint = p.BackPoint()
	}
	fmt.Fprintf(os.Stderr, "  Non-cutting travel distance before optimization: %f\n", travelDistance)

	// Create a linked list of all paths not used so we can remove them once
	// they are used
	oldPaths := new(list.List)
	for _, p := range opc.paths[1:] {
		oldPaths.PushBack(p)
	}

	// Now do a simple N^2 greedy algorithm to add paths to the new list based
	// on the smallest distance.
	newPaths := []*Path{opc.paths[0]}
	travelDistance = 0
	lastPoint = opc.paths[0].Back().P2()
	for oldPaths.Len() != 0 {
		bestDistance := math.MaxFloat64
		bestDistanceElem := (*list.Element)(nil)
		for p := oldPaths.Front(); p != nil; p = p.Next() {
			d := lastPoint.DistanceFrom(*p.Value.(*Path).FrontPoint())
			if d < bestDistance {
				bestDistance = d
				bestDistanceElem = p
			}
			d = lastPoint.DistanceFrom(*p.Value.(*Path).BackPoint())
			if d < bestDistance {
				p.Value.(*Path).Reverse()
				bestDistance = d
				bestDistanceElem = p
			}
		}
		newPaths = append(newPaths, bestDistanceElem.Value.(*Path))
		lastPoint = bestDistanceElem.Value.(*Path).Back().P2()
		travelDistance += bestDistance
		oldPaths.Remove(bestDistanceElem)
	}
	opc.paths = newPaths
	fmt.Fprintf(os.Stderr, "  Non-cutting travel distance after optimization: %f\n", travelDistance)
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
	for _, path := range opc.paths {
		if path.IsClosed() {
			validPaths = append(validPaths, path)
		}
	}
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

type OptimizedRenderOutput struct {
	cuts  OptimizedPathCollection
	mark1 OptimizedPathCollection
	mark2 OptimizedPathCollection
}

func (me *OptimizedRenderOutput) Optimize() {
	fmt.Fprintf(os.Stderr, "Optimizing cut paths\n")
	me.cuts.Optimize()

	fmt.Fprintf(os.Stderr, "Optimizing mark1 paths\n")
	me.mark1.Optimize()
	me.mark1.RemoveUnclosedPaths()

	fmt.Fprintf(os.Stderr, "Optimizing mark2 paths\n")
	me.mark2.Optimize()
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


////////////////////////////////////////////////////////////////////////////
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
	LightToDark	ScaleMode = "ltd"	// light/dark = 1/phi
	DarkToLight	ScaleMode = "dtl"	// dark/light = 1/phi
	LightToTotal	ScaleMode = "ltt"	// light/(light+dark)
	DarkToTotal	ScaleMode = "dtt"	// dark/(light+dark) = 1/phi
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
	var wg sync.WaitGroup
	var mutex sync.Mutex

	fmt.Fprintf(os.Stderr, "Starting primitive count: %d\n", len(r))
	for i := 0; i < levels; i++ {
		rNext := make([]PenrosePrimitive, 0, 3*len(r))
		for _, shape := range r {
			// rNext = append(rNext, shape.Deflate()...)
			wg.Add(1)
			go func(s PenrosePrimitive) {
				defer wg.Done()
				newShapes := s.Deflate()
				mutex.Lock()
				rNext = append(rNext, newShapes...)
				mutex.Unlock()
			}(shape)
		}
		wg.Wait()
		r = rNext
		// fmt.Fprintf(os.Stderr, "Primitive count after deflation %d: %d\n", i+1, len(r))
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
	fmt.Fprintf(os.Stderr, "Primitive count after cull: %d\n", len(r))
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

////////////////////////////////////////////////////////////////////////////
func (me *OptimizedRenderOutput) GenerateScaledImages(outputDir string, mode ScaleMode) {
    // Ensure the output directory exists
    if err := os.MkdirAll(outputDir, 0755); err != nil {
        fmt.Fprintf(os.Stderr, "Error creating output directory: %v\n", err)
        return
    }

    // Filter paths by length
    filteredMark1 := me.mark1.Filtered(0.09)

    // Group filtered paths by their rounded lengths
    lengthGroups := make(map[float64][]*Path)
    var uniqueLengths []float64
    for _, path := range filteredMark1.paths {
        l := math.Round(path.TotalLength()*1e6) / 1e6
        if _, exists := lengthGroups[l]; !exists {
            uniqueLengths = append(uniqueLengths, l)
        }
        lengthGroups[l] = append(lengthGroups[l], path)
    }
    sort.Float64s(uniqueLengths)

    // Process each group
    for groupIdx, length := range uniqueLengths {
        // Create a fresh copy of paths for this group
        groupPaths := &OptimizedPathCollection{}
        for _, path := range lengthGroups[length] {
            // Create a deep copy of the path
            newPath := &Path{segs: list.New()}
            for e := path.segs.Front(); e != nil; e = e.Next() {
                switch seg := e.Value.(type) {
                case *CutLine:
                    newPath.PushBack(&CutLine{A: seg.A, B: seg.B})
                case *MarkArc:
                    newPath.PushBack(&MarkArc{
                        C: seg.C, A: seg.A, B: seg.B,
                        R: seg.R, Offset: seg.Offset, Reversed: seg.Reversed,
                    })
                }
            }
            groupPaths.paths = append(groupPaths.paths, newPath)
        }

        // Calculate areas
        lightArea := 0.0
        for _, path := range groupPaths.paths {
            if path.IsClosed() {
                lightArea += path.Area()
            }
        }
        decagonArea := DecagonArea(1.0)
        darkArea := decagonArea - lightArea
        totalArea := decagonArea

        // Calculate current ratio and scale factor based on mode
        targetRatio := 1.0 / math.Phi
        currentRatio := 0.0
        scaleFactor := 1.0

        switch mode {
        case LightToDark:
            currentRatio = lightArea / darkArea
            scaleFactor = math.Sqrt(targetRatio / currentRatio)
        case DarkToLight:
            currentRatio = darkArea / lightArea
            scaleFactor = math.Sqrt(targetRatio / currentRatio)
        case LightToTotal:
            currentRatio = lightArea / totalArea
            scaleFactor = math.Sqrt(targetRatio / currentRatio)
        case DarkToTotal:
            currentRatio = darkArea / totalArea
            scaleFactor = math.Sqrt(targetRatio / currentRatio)
        }

        fmt.Printf("In mode %s precalculated ratio %.6f and Scale Factor of %.6f\n", mode, currentRatio, scaleFactor)

        // Apply scaling based on the specified mode
        groupPaths.ScaleToGoldenRatio(mode)

        // Create output file
        filename := fmt.Sprintf("%s/group%d_%s.svg", outputDir, groupIdx+1, mode)
        file, err := os.Create(filename)
        if err != nil {
            fmt.Printf("Error creating file: %v\n", err)
            continue
        }
        defer file.Close()

        // Generate SVG
        s := NewSVG(file)
        bounds := geom.Rect{geom.Coord{-1.5, -1.5}, geom.Coord{1.5, 1.5}}
        s.Start(bounds, DEFAULT_STYLE)

        // Draw reference decagon
        decagonPoints := CreateCenteredDecagon(1.0)
        s.StartPath(decagonPoints[0], "stroke:black;stroke-width:0.002;fill:none")
        for i := 1; i < 10; i++ {
            s.PathLineTo(decagonPoints[i])
        }
        s.PathLineTo(decagonPoints[0])
        s.EndPath()

        // Draw scaled paths
        color := fmt.Sprintf("hsl(%d, 100%%, 50%%)", groupIdx*180)
        for _, path := range groupPaths.paths {
            path.Draw(s, fmt.Sprintf("stroke: %s; stroke-width:0.002", color))
        }
        s.End()

        fmt.Printf("Generated %s\n\tafter scaling %.6f\n", filename, scaleFactor)
    }
}


func main() {
	shapes := Sun()
	bounds := geom.Rect{geom.Coord{-300, -200}, geom.Coord{300, 200}}
	bounds.Scale(1.0/150.0, 1.0/150.0)

	shapes = DeflatePenrosePrimitives(shapes, DEFLATE_LEVEL)
	shapes = CullShapes(shapes, &bounds)
	
	ro := &RenderOutput{}
	for _, shape := range shapes {
		shape.Render(ro)
	}
	ro.RemoveDuplicates()
	oro := ro.Optimize()

    oro.GenerateScaledImages("output", LightToDark)
    oro.GenerateScaledImages("output", DarkToLight)
    oro.GenerateScaledImages("output", LightToTotal)
    oro.GenerateScaledImages("output", DarkToTotal)
}