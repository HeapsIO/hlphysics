package physics.collision.narrowphase;

typedef Edges = hl.CArray<Edge>;

class TrianglesIterator {
	public var i(default, null) : Int;
	var l : Int;
	var a : Triangles;
	public inline function new(a, i) {
		this.i = i;
		this.a = a;
		this.l = this.a.length;
	}
	public inline function hasNext() {
		return i < l;
	}
	public inline function next() : Triangle {
		return a.get(i++);
	}
}

class Triangles {
	var container : hl.NativeArray<Triangle>;
	var capacity : Int;
	public var length(default, null) : Int;

	public function new( capacity:Int ) {
		container = new hl.NativeArray(capacity);
		this.capacity = capacity;
		this.length = 0;
	}

	public inline function get( idx : Int ) : Triangle {
		return container[idx];
	}

	public inline function set( idx : Int, t : Triangle ) : Triangle {
		return container[idx] = t;
	}

	public inline function push( t : Triangle ) {
		container[length++] = t;
	}

	public inline function pop() {
		var t = container[--length];
		container[length] = null;
		return t;
	}

	public inline function clear() {
		for ( i in 0...length )
			container[i] = null;
		length = 0;
	}

	public inline function iterator() : TrianglesIterator {
		return new TrianglesIterator(this, 0);
	}
}

@:struct
class Edge {
	public var neighbourTriangle : Triangle;
	public var neighbourEdge : Int;
	public var startIdx : Int;
}

@:struct
class Triangle {
	static inline final MIN_TRIANGLE_AREA = 1.0e-10;
	static inline final BARYCENTRIC_EPSILON = 1.0e-3;

	@:packed public var edge0 : Edge;
	@:packed public var edge1 : Edge;
	@:packed public var edge2 : Edge;
	@:packed public var normal : Vec3;
	@:packed public var centroid : Vec3;
	public var closestLenSq : Scalar = Math.SCALAR_MAX;
	public var lambdaX : Scalar;
	public var lambdaY : Scalar;
	public var lambdaRelativeTo0 : Bool;
	public var closestPointInterior : Bool = false;
	public var removed : Bool = false;
	public var inQueue : Bool = false;

	public function new() {}

	public function initialize( idx0: Int, idx1: Int, idx2 : Int, positions : hl.CArray<Vec3>) {
		Assert.t(idx0 != idx1 && idx0 != idx2 && idx1 != idx2);
		removed = false;
		inQueue = false;
		closestPointInterior = false;
		closestLenSq = Math.SCALAR_MAX;

		edge0.startIdx = idx0;
		edge1.startIdx = idx1;
		edge2.startIdx = idx2;

		edge0.neighbourTriangle = null;
		edge1.neighbourTriangle = null;
		edge2.neighbourTriangle = null;

		var y0 = positions[idx0].clone();
		var y1 = positions[idx1].clone();
		var y2 = positions[idx2].clone();

		centroid.load( (y0 + y1 + y2) * (1.0/3.0) );

		var y10 = y1 - y0;
		var y20 = y2 - y0;
		var y21 = y2 - y1;

		var y20DotY20 = y20.dot(y20);
		var y21DotY21 = y21.dot(y21);
		if ( y20DotY20 < y21DotY21) {
			normal.load(y10.cross(y20));

			var normalLenSq = normal.lengthSq();
			if( normalLenSq > MIN_TRIANGLE_AREA ) {
				var cDotN = centroid.dot(normal);
				closestLenSq = Math.abs(cDotN) * cDotN / normalLenSq;

				var y10DotY10 = y10.lengthSq();
				var y10DotY20 = y10.dot(y20);
				var determinant = y10DotY10 * y20DotY20 - y10DotY20 * y10DotY20;
				if ( determinant > 0.0 ) {
					var y0DotY10 = y0.dot(y10);
					var y0DotY20 = y0.dot(y20);
					var l0 = (y10DotY20 * y0DotY20 - y20DotY20 * y0DotY10) / determinant;
					var l1 = (y10DotY20 * y0DotY10 - y10DotY10 * y0DotY20) / determinant;
					lambdaX = l0;
					lambdaY = l1;
					lambdaRelativeTo0 = true;

					if ( l0 > -BARYCENTRIC_EPSILON && l1 > -BARYCENTRIC_EPSILON && l0 + l1 < 1.0 + BARYCENTRIC_EPSILON)
						closestPointInterior = true;
				}
			}
		} else {
			normal.load(y10.cross(y21));

			var normalLenSq = normal.lengthSq();
			if( normalLenSq > MIN_TRIANGLE_AREA ) {
				var cDotN = centroid.dot(normal);
				closestLenSq = Math.abs(cDotN) * cDotN / normalLenSq;

				var y10DotY10 = y10.lengthSq();
				var y10DotY21 = y10.dot(y21);
				var determinant = y10DotY10 * y21DotY21 - y10DotY21 * y10DotY21;
				if ( determinant > 0.0 ) {
					var y1DotY10 = y1.dot(y10);
					var y1DotY21 = y1.dot(y21);
					var l0 = ( y21DotY21 * y1DotY10 - y10DotY21 * y1DotY21 ) / determinant;
					var l1 = ( y10DotY21 * y1DotY10 - y10DotY10 * y1DotY21 ) / determinant;
					lambdaX = l0;
					lambdaY = l1;
					lambdaRelativeTo0 = false;

					if ( l0 > -BARYCENTRIC_EPSILON && l1 > -BARYCENTRIC_EPSILON && l0 + l1 < 1.0 + BARYCENTRIC_EPSILON)
						closestPointInterior = true;
				}
			}
		}
	}

	public inline function isFacing(position : Vec3) : Bool {
		Assert.t(!removed);
		return normal.dot(position - centroid) > 0.0;
	}

	public inline function isFacingOrigin() {
		Assert.t(!removed);
		return normal.dot(centroid) < 0.0;
	}

	public inline function getNextEdge(idx : Int) : Edge {
		var idx = (idx + 1) % 3;
		return getEdge(idx);
	}

	public inline function getEdge( idx: Int) : Edge {
		return switch(idx) {
			case 0: edge0;
			case 1: edge1;
			case 2: edge2;
			default: throw "assert";
		}
	}
}

@:struct
private class FreeBlock {
	public var nextFree : FreeBlock;
}

class TriangleFactory {
	var container : hl.CArray<Triangle>;
	var nextFree : FreeBlock = null;
	var cursor : Int;

	public function new() {
		container = hl.CArray.alloc(Triangle, EPAConvexHullBuilder.MAX_TRIANGLES);
		clear();
	}

	public function clear() {
		nextFree = null;
		cursor = 0;
	}

	public function createTriangle(idx0, idx1, idx2, positions) : Triangle {
		var t : Triangle = null;
		if ( nextFree != null ) {
			t = hl.Api.unsafeCast(nextFree);
			nextFree = nextFree.nextFree;
		} else {
			if ( cursor >= EPAConvexHullBuilder.MAX_TRIANGLES)
				return null;
			t = container[cursor++];
		}
		t.initialize(idx0, idx1, idx2, positions);
		return t;
	}

	public function freeTriangle(t : Triangle ) {
		Assert.t(t.removed);
		Assert.t(t.edge0.neighbourTriangle == null);
		Assert.t(t.edge1.neighbourTriangle == null);
		Assert.t(t.edge2.neighbourTriangle == null);
		var freeBlock : FreeBlock = hl.Api.unsafeCast(t);
		freeBlock.nextFree = nextFree;
		nextFree = freeBlock;
	}
}

class TriangleQueue {
	var triangles : Triangles;

	public function new() {
		this.triangles = new Triangles(EPAConvexHullBuilder.MAX_TRIANGLES);
	}

	public function reset() {
		triangles.clear();
	}

	public static function trianglePredicat( t1 :Triangle, t2 : Triangle ) : Bool {
		return t1.closestLenSq > t2.closestLenSq;
	}

	public function push( t : Triangle ) {
		triangles.push(t);
		t.inQueue = true;

		var count = triangles.length;
		var current = count - 1;
		while (current > 0) {
			var currentElem = triangles.get(current);

			var parent = (current - 1) >> 1;
			var parentElem = triangles.get(parent);

			if (trianglePredicat(parentElem, currentElem)) {
				triangles.set(current, parentElem);
				triangles.set(parent, currentElem);
				current = parent;
			} else
				break;
		}
	}

	public function peekClosest() {
		return triangles.get(0);
	}

	public function popClosest() : Triangle {

		inline function swap( a, b ) {
			var ea = triangles.get(a);
			var eb = triangles.get(b);
			triangles.set(a, eb);
			triangles.set(b, ea);
		}

		swap(0, triangles.length - 1);

		var count = triangles.length - 1;

		var largest = 0;
		while(true) {
			var child = (largest << 1) + 1;
			if (child >= count)
				break;

			var prevLargest = largest;
			if (trianglePredicat(triangles.get(largest), triangles.get(child)))
				largest = child;

			++child;
			if (child < count && trianglePredicat(triangles.get(largest), triangles.get(child)))
				largest = child;

			if (prevLargest == largest)
				break;

			swap(prevLargest, largest);
		}

		return triangles.pop();
	}
}

@:struct
class StackEntry {
	public var triangle : Triangle;
	public var edge: Int;
	public var iter : Int;
}

class EPAConvexHullBuilder {
	public static inline final MAX_EDGE_LENGTH = 128;
	public static inline final MAX_TRIANGLES = 256;

	var positions : hl.CArray<Vec3>;
	public var newTriangles : Triangles;
	var factory : TriangleFactory;
	var triangleQueue : TriangleQueue;

	var stack : hl.CArray<StackEntry>;
	var edges : hl.CArray<Edge>;
	var numEdges : Int;

	public function new() {
		factory = new TriangleFactory();
		triangleQueue = new TriangleQueue();
		newTriangles = new Triangles(MAX_EDGE_LENGTH);
		stack = hl.CArray.alloc(StackEntry, MAX_EDGE_LENGTH);
		edges = hl.CArray.alloc(Edge, MAX_EDGE_LENGTH);
	}

	public function reset( positions ) {
		this.positions = positions;
		triangleQueue.reset();
		newTriangles.clear();
		numEdges = 0;
	}

	public function initialize( idx1 : Int, idx2 : Int, idx3 : Int ) {
		factory.clear();

		var t1 = createTriangle(idx1, idx2, idx3);
		var t2 = createTriangle(idx1, idx3, idx2);

		linkTriangle(t1, 0, t2, 2);
		linkTriangle(t1, 1, t2, 1);
		linkTriangle(t1, 2, t2, 0);

		triangleQueue.push(t1);
		triangleQueue.push(t2);
	}

	public function hasNextTriangle() {
		return @:privateAccess triangleQueue.triangles.length != 0;
	}

	public function peekClosestTriangleInQueue() {
		return triangleQueue.peekClosest();
	}

	public function popClosestTriangleFromQueue() {
		return triangleQueue.popClosest();
	}

	public inline function findFacingTriangle( pos : Vec3, out : { bestDistSq : Scalar } ) : Triangle {
		var best : Triangle = null;
		var bestDistSq = 0.0;

		for ( t in @:privateAccess triangleQueue.triangles ) {
			if ( !t.removed ) {
				var dot = t.normal.dot(pos - t.centroid);
				if( dot > 0.0 ) {
					var distSq = dot * dot / t.normal.lengthSq();
					if( distSq > bestDistSq ) {
						best = t;
						bestDistSq = distSq;
					}
				}
			}
		}

		out.bestDistSq = bestDistSq;
		return best;
	}

	public function addPoint( facingTriangle : Triangle, idx : Int, closestDistSq : Scalar ) {
		var pos = positions[idx];

		newTriangles.clear();
		var triangles = newTriangles;

		numEdges = 0;
		if (!findEdge(facingTriangle, pos, edges))
			return false;

		for ( i in 0...numEdges ) {
			var nt = createTriangle(edges[i].startIdx, edges[(i + 1) % numEdges].startIdx, idx);
			if( nt == null )
				return false;
			triangles.push(nt);

			if ( (nt.closestPointInterior && nt.closestLenSq < closestDistSq) || nt.closestLenSq < 0.0 )
				triangleQueue.push(nt);
		}

		for ( i in 0...numEdges ) {
			linkTriangle(triangles.get(i), 0, edges[i].neighbourTriangle, edges[i].neighbourEdge);
			linkTriangle(triangles.get(i), 1, triangles.get((i + 1) % numEdges), 2);
		}

		return true;
	}

	function findEdge( facingTriangle : Triangle, vertex : Vec3, edges : Edges ) : Bool {
		Assert.t(numEdges == 0);
		Assert.t(facingTriangle.isFacing(vertex));

		facingTriangle.removed = true;

		var curStackPos = 0;
		{
			var s = stack[0];
			s.triangle = facingTriangle;
			s.edge = 0;
			s.iter = -1;
		}

		var nextExpectedStartIdx = -1;

		while(true) {
			var cur = stack[curStackPos];

			if ( ++cur.iter >= 3 ) {
				unlinkTriangle(cur.triangle);

				if ( --curStackPos < 0 )
					break;
			} else {
				var e = cur.triangle.getEdge((cur.edge + cur.iter) % 3);
				var n = e.neighbourTriangle;
				if ( n != null && !n.removed ) {
					if ( n.isFacing(vertex)) {
						n.removed = true;
						curStackPos++;
						var newEntry = stack[curStackPos];
						newEntry.triangle = n;
						newEntry.edge = e.neighbourEdge;
						newEntry.iter = 0;
					} else {
						if ( e.startIdx != nextExpectedStartIdx && nextExpectedStartIdx != -1)
							return false;

						nextExpectedStartIdx = n.getEdge(e.neighbourEdge).startIdx;

						edges.unsafeSet(numEdges++, e);
					}
				}
			}
		}

		Assert.t(numEdges == 0 || edges[0].startIdx == nextExpectedStartIdx);

		return numEdges >= 3;
	}

	public inline function freeTriangle( t : Triangle ) {
		factory.freeTriangle(t);
	}

	function unlinkTriangle( t : Triangle ) {
		for (i in 0...3) {
			var edge = t.getEdge(i);
			if ( edge.neighbourTriangle != null ) {
				var neighbourEdge = edge.neighbourTriangle.getEdge(edge.neighbourEdge);

				Assert.t(neighbourEdge.neighbourTriangle == t);
				Assert.t(neighbourEdge.neighbourEdge == i);

				neighbourEdge.neighbourTriangle = null;
				edge.neighbourTriangle = null;
			}
		}

		if ( !t.inQueue )
			freeTriangle(t);
	}

	static function linkTriangle( t1 : Triangle, edge1 : Int, t2 : Triangle, edge2 : Int ) {
		Assert.t(edge1 >= 0 && edge1 < 3);
		Assert.t(edge2 >= 0 && edge2 < 3);
		var e1 = t1.getEdge(edge1);
		var e2 = t2.getEdge(edge2);

		Assert.t(e1.neighbourTriangle == null);
		Assert.t(e2.neighbourTriangle == null);

		Assert.t(e1.startIdx == t2.getNextEdge(edge2).startIdx);
		Assert.t(e2.startIdx == t1.getNextEdge(edge1).startIdx);

		e1.neighbourTriangle = t2;
		e1.neighbourEdge = edge2;
		e2.neighbourTriangle = t1;
		e2.neighbourEdge = edge1;
	}

	inline function createTriangle( idx1 : Int, idx2 : Int, idx3 : Int ) : Triangle {
		return factory.createTriangle(idx1, idx2, idx3, positions);
	}
}
