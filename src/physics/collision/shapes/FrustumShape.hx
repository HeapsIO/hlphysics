package physics.collision.shapes;

class FrustumShape extends ConvexHullShape {
	public static inline final PLANE_COUNT = 6;

	/**
		The indices should describe 6 quad faces, with 2 consecutive triangles per face.
	**/
	public inline function new( points : Array<Single>, indices : Array<Int> ) {
		super(points, indices);
	}

	override public function toString() {
		return "Frustum";
	}

	public function getPlanes( scale : Vec3, transform : Mat, result : StaticArray<Plane> ) {
		var points = this.points;
		var indices = this.indices;
		var pointCount = Std.int(points.length / 3);
		var center = new Vec3();
		for( i in 0...pointCount ) {
			var pos = i * 3;
			center.x += points[pos];
			center.y += points[pos + 1];
			center.z += points[pos + 2];
		}
		var invPointCount = 1.0 / pointCount;
		center.x *= invPointCount * scale.x;
		center.y *= invPointCount * scale.y;
		center.z *= invPointCount * scale.z;
		center.transform(transform);

		var p0 = new Vec3();
		var p1 = new Vec3();
		var p2 = new Vec3();
		inline function loadPoint( out : Vec3, index : Int ) {
			var pos = index * 3;
			out.set(points[pos] * scale.x, points[pos + 1] * scale.y, points[pos + 2] * scale.z);
			out.transform(transform);
		}
		for( i in 0...PLANE_COUNT ) {
			var indexPos = i * 6;
			loadPoint(p0, indices[indexPos]);
			loadPoint(p1, indices[indexPos + 1]);
			loadPoint(p2, indices[indexPos + 2]);
			result.get(i).setFromPoints(p0, p1, p2, center);
		}
	}

	#if heaps
	static var HEAPS_INDICES = [
		4, 0, 3,  4, 3, 7, // left
		1, 5, 6,  1, 6, 2, // right
		4, 5, 1,  4, 1, 0, // top
		3, 2, 6,  3, 6, 7, // bottom
		0, 1, 2,  0, 2, 3, // near
		5, 4, 7,  5, 7, 6, // far
	];

	/**
		8 corners, near face first then far face, both in clockwise order,
		e.g. NTL, NTR, NBR, NBL, FTL, FTR, FBR, FBL.
	**/
	public static function fromHeapsCorners( corners : Array<h3d.Vector> ) : FrustumShape {
		if( corners.length != 8 )
			throw "Invalid corners length for FrustumShape " + corners.length;
		var points : Array<Single> = [];
		for( c in corners ) {
			points.push(c.x);
			points.push(c.y);
			points.push(c.z);
		}
		return new FrustumShape(points, HEAPS_INDICES);
	}
	#end
}
