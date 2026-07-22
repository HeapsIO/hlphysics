package physics.collision.shapes;

class TriangleSupport extends ConvexSupport {
	@:packed public var v0 : Vec3;
	@:packed public var v1 : Vec3;
	@:packed public var v2 : Vec3;

	public inline function init( shape : Shape, scale : Vec3 ) {
		var tri : TriangleShape = hl.Api.unsafeCast(shape);
		this.v0.load(tri.v0.multiplied(scale));
		this.v1.load(tri.v1.multiplied(scale));
		this.v2.load(tri.v2.multiplied(scale));
	}

	public inline function getSupportWithMargin( dir : Vec3, out : Vec3 ) {
		getSupport(dir, out);
		var len = dir.length();
		if( len > 0.0 )
			out.add(dir.scaled(getMargin() / len));
	}

	public inline function getSupport( dir : Vec3, out : Vec3 ) {
		var d1 = v0.dot(dir);
		var d2 = v1.dot(dir);
		var d3 = v2.dot(dir);
		if( d1 > d2 ) {
			if( d1 > d3 )
				out.load(v0);
			else
				out.load(v2);
		} else {
			if( d2 > d3 )
				out.load(v1);
			else
				out.load(v2);
		}
	}

	public inline function getMargin() : Scalar {
		return 0.0;
	}
}

class TriangleShape extends ConvexShape {

	@:packed public var v0 : Vec3;
	@:packed public var v1 : Vec3;
	@:packed public var v2 : Vec3;

	public inline function new( v0 : Vec3, v1 : Vec3, v2 : Vec3 ) {
		shapeType = Triangle;
		this.v0 = v0;
		this.v1 = v1;
		this.v2 = v2;
	}

	public function toString() {
		return "Triangle";
	}

	public inline function getLocalBounds() {
		var min = Vec3.min(v0, Vec3.min(v1, v2));
		var max = Vec3.max(v0, Vec3.max(v1, v2));
		return new AABB(min, max);
	}

	public inline function getLocalBoundsToBuffer( out : AABB ) {
		out.load(getLocalBounds());
	}

	public inline function getSurfaceNormal( localPos : Vec3 ) : Vec3 {
		var cross = (v1 - v0).cross(v2 - v0);
		var len = cross.length();
		if( len != 0.0 ) {
			cross.scale(1.0 / len);
		} else {
			cross.set(0.0, 0.0, 1.0);
		}
		return cross;
	}

	public inline function raycast( ray : Ray, scale : Vec3, transform : Mat, infos : HitResult ) : Bool {
		var invTransform = transform.getInverse();
		var torigin = ray.origin.transformed(invTransform);
		var tdirection = ray.direction.transformed3x3(invTransform);
		var fraction = Math.rayTriangle(torigin, tdirection, v0.multiplied(scale), v1.multiplied(scale), v2.multiplied(scale));
		if ( fraction < Math.SCALAR_MAX ) {
			infos.position.load(ray.getPoint(fraction));
			infos.normal.load(getSurfaceNormal(torigin + tdirection * fraction).transformed3x3(transform));
			infos.fraction = fraction;
			return true;
		}
		return false;
	}

	public inline function isScaleValid( scale : Vec3 ) : Bool {
		return !ScaleHelper.isNearZero(scale);
	}

	public inline function makeScaleValid( scale : Vec3 ) {
		scale.load(ScaleHelper.makeNonZero(scale));
	}

	public inline function getSupportClass() {
		return TriangleSupport;
	}

	public inline function scaled( scale : Vec3 ) : TriangleShape {
		return new TriangleShape(v0.multiplied(scale), v1.multiplied(scale), v2.multiplied(scale));
	}
}
