package physics.collision.shapes;

class MeshShape extends Shape {

	var points : Array<Single>;
	var indices : Array<Int>;
	var localBounds : AABB;
	public var tree(default, null) : AABBTree;
	var triangles : StaticArray<TriangleShape>;

	public inline function new( points : Array<Single>, indices : Array<Int> ) {
		shapeType = Mesh;
		this.points = points;
		this.indices = indices;
		var firstPoint = new Vec3(points[0], points[1], points[2]);
		var bounds = new AABB(firstPoint, firstPoint);
		for( i in 1...Std.int(points.length / 3) ) {
			var pos = i * 3;
			bounds.addPos(new Vec3(points[pos++], points[pos++], points[pos]));
		}
		this.localBounds = bounds;
		var triCount = Math.floor(indices.length / 3);
		tree = new AABBTree(0.0, triCount);
		triangles = new StaticArray(TriangleShape, triCount);
		for( triIndex in 0...triCount ) {
			var tri = triangles.pushEmpty();
			var i = triIndex * 3;
			var i0 = indices[i++] * 3;
			var p0 = new Vec3(points[i0++], points[i0++], points[i0]);
			var i1 = indices[i++] * 3;
			var p1 = new Vec3(points[i1++], points[i1++], points[i1]);
			var i2 = indices[i] * 3;
			var p2 = new Vec3(points[i2++], points[i2++], points[i2]);
			tri.v0.load(p0);
			tri.v1.load(p1);
			tri.v2.load(p2);
			tree.addBody(tri.getLocalBounds(), triIndex);
		}
	}

	public function toString() {
		return "Mesh";
	}

	public inline function getLocalBounds() {
		return localBounds;
	}

	public inline function getLocalBoundsToBuffer( out : AABB ) {
		out.load(getLocalBounds());
	}

	public inline function getSurfaceNormal( localPos : Vec3 ) : Vec3 {
		throw "Not implemented";
	}

	public function raycast( ray : Ray, scale : Vec3, transform : Mat, infos : HitResult ) : Bool {
		throw "Should not be called directly"; // See MeshAlgorithm.raycast
	}

	public inline function getTriangle( triIndex : Int ) : TriangleShape {
		return triangles.get(triIndex);
	}

	public inline function isScaleValid( scale : Vec3 ) : Bool {
		return !ScaleHelper.isNearZero(scale);
	}

	public inline function makeScaleValid( scale : Vec3 ) {
		scale.load(ScaleHelper.makeNonZero(scale));
	}

	#if heaps
	public static function fromHeaps( poly : h3d.col.PolygonBuffer ) : MeshShape {
		var shape = new MeshShape(cast @:privateAccess poly.buffer, cast @:privateAccess poly.indexes);
		return shape;
	}
	#end
}
