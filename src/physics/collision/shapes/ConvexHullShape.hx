package shiro.physics.collision.shapes;

class ConvexHullSupport extends ConvexSupport {
	public var convex : ConvexHullShape;
	@:packed public var scale : Vec3;

	public inline function init( shape : Shape, scale : Vec3 ) {
		var convex : ConvexHullShape = hl.Api.unsafeCast(shape);
		this.convex = convex;
		this.scale.load(scale);
	}

	public inline function getSupportWithMargin( dir : Vec3, out : Vec3 ) {
		return getSupport(dir, out);
	}

	public inline function getSupport( dir : Vec3, out : Vec3 ) {
		var bestDot = Math.SCALAR_MIN;
		out.set(0.0, 0.0, 0.0);
		var points = @:privateAccess convex.points;
		var scale = scale;
		for ( i in 0...Std.int(points.length / 3) ) {
			var pos = i * 3;
			var p = new Vec3(points[pos++], points[pos++], points[pos]) * scale;
			var dot = p.dot(dir);
			if ( dot > bestDot ) {
				bestDot = dot;
				out.load(p);
			}
		}
	}

	public inline function getMargin() : Scalar {
		return 0.0;
	}
}

class ConvexHullShape extends ConvexPolyhedronShape {

	// always use single precision for these points data (smaller footprint and no reinterpret polygon buffer data)
	var points : Array<Single>;
	var indices : Array<Int>;
	var localBounds : AABB;

	public inline function new( points : Array<Single>, indices : Array<Int> ) {
		shapeType = ConvexHull;
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
		return "ConvexHull";
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
		var invTransform = transform.getInverse();
		var torigin = ray.origin.transformed(invTransform);
		var tdirection = ray.direction.transformed3x3(invTransform);
		var best = Math.SCALAR_MAX;
		var bestTriIndex = 0;
		var triCount = Math.floor(indices.length / 3);
		for( triIndex in 0...triCount ) {
			var tri = getTriangle(triIndex).scaled(scale);
			var t = Math.rayTriangle(torigin, tdirection, tri.v0, tri.v1, tri.v2);
			if( t < best ) {
				best = t;
				bestTriIndex = triIndex;
			}
		}

		if ( best < Math.SCALAR_MAX ) {
			infos.fraction = best;
			infos.position.load(ray.getPoint(best));
			var tri = getTriangle(bestTriIndex).scaled(scale);
			infos.normal.load(tri.getSurfaceNormal(Vec3.zero()).transformed3x3(transform));
			return true;
		}

		return false;
	}

	inline function getTriangle( triIndex : Int ) : TriangleShape {
		var indices = this.indices;
		var points = this.points;
		var i = triIndex * 3;
		var i0 = indices[i++] * 3;
		var v0 = new Vec3(points[i0++], points[i0++], points[i0]);
		var i1 = indices[i++] * 3;
		var v1 = new Vec3(points[i1++], points[i1++], points[i1]);
		var i2 = indices[i++] * 3;
		var v2 = new Vec3(points[i2++], points[i2++], points[i2]);
		return new TriangleShape(v0, v1, v2);
	}

	public inline function isScaleValid( scale : Vec3 ) : Bool {
		return !ScaleHelper.isNearZero(scale);
	}

	public inline function makeScaleValid( scale : Vec3 ) {
		scale.load(ScaleHelper.makeNonZero(scale));
	}

	public inline function getSupportClass() {
		return ConvexHullSupport;
	}

	#if heaps
	public static function fromHeaps( poly : h3d.col.PolygonBuffer ) : ConvexHullShape {
		var shape = new ConvexHullShape(cast @:privateAccess poly.buffer, cast @:privateAccess poly.indexes);
		return shape;
	}
	#end
}
