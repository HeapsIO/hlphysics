package physics.collision.shapes;

class FrustumShape extends ConvexHullShape {

	public inline function new( points : Array<Single>, indices : Array<Int> ) {
		super(points, indices);
	}

	override public function toString() {
		return "Frustum";
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
