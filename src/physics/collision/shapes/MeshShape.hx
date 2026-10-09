package physics.collision.shapes;

class MeshShape extends Shape {

	var points : Array<Single>;
	var indices : Array<Int>;
	var localBounds : AABB;
	var tree(default, null) : Null<AABBTree>;

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
	}

	public function toString() {
		return "Mesh";
	}

	override function buildLater() : Void {
		if( tree != null )
			return;
		// Might be called from a thread, assign tree at the end
		var triCount = Math.floor(indices.length / 3);
		var t = new AABBTree(0.0, 2 * triCount);
		var bounds = new StaticArray(AABB, triCount);
		for( i in 0...triCount ) {
			var aabb = bounds.pushEmpty();
			getTriangle(i).getLocalBoundsToBuffer(aabb);
		}
		t.build(bounds);
		tree = t;
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
		var i = triIndex * 3;
		var i0 = indices[i++] * 3;
		var v0 = new Vec3(points[i0++], points[i0++], points[i0]);
		var i1 = indices[i++] * 3;
		var v1 = new Vec3(points[i1++], points[i1++], points[i1]);
		var i2 = indices[i] * 3;
		var v2 = new Vec3(points[i2++], points[i2++], points[i2]);
		return new TriangleShape(v0, v1, v2);
	}

	public inline function getTriangleToBuffer( triIndex : Int, out : TriangleShape ) {
		var tri = getTriangle(triIndex);
		out.v0.load(tri.v0);
		out.v1.load(tri.v1);
		out.v2.load(tri.v2);
	}

	public inline function isScaleValid( scale : Vec3 ) : Bool {
		return !ScaleHelper.isNearZero(scale);
	}

	public inline function makeScaleValid( scale : Vec3 ) {
		scale.load(ScaleHelper.makeNonZero(scale));
	}

	/**
		Visits `tree` if it has been built, otherwise brute-force over all triangles.
	**/
	public function walk( visitor : TreeVisitor, tmpNode : TreeNode ) : Void {
		if ( tree != null ) {
			tree.walkTree(visitor);
			return;
		}
		var triCount = Math.floor(indices.length / 3);
		for ( i in 0...triCount ) {
			getTriangle(i).getLocalBoundsToBuffer(tmpNode.aabb);
			tmpNode.bodyID = i;
			if ( !visitor.visitBody(tmpNode) )
				break;
		}
	}

	#if heaps
	public static function fromHeaps( poly : h3d.col.PolygonBuffer ) : MeshShape {
		var shape = new MeshShape(cast @:privateAccess poly.buffer, cast @:privateAccess poly.indexes);
		return shape;
	}
	#end
}
