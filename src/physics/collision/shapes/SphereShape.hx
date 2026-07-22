package shiro.physics.collision.shapes;

class SphereSupport extends ConvexSupport {
	public var radius : Scalar;

	public inline function init( shape : Shape, scale : Vec3 ) {
		var sphere : SphereShape = hl.Api.unsafeCast(shape);
		this.radius = sphere.radius * scale.x;
	}

	public inline function getSupportWithMargin( dir : Vec3, out : Vec3 ) {
		var len = dir.length();
		if ( len > 0.0 )
			out.load(dir * ( radius / len ));
		else
			out.set(0.0, 0.0, 0.0);
	}

	public inline function getSupport( dir : Vec3, out : Vec3 ) {
		out.set(0.0, 0.0, 0.0);
	}

	public inline function getMargin() : Scalar {
		return radius;
	}
}

class SphereShape extends ConvexShape {
	public var radius : Scalar;

	public inline function new( radius : Scalar ) {
		shapeType = Sphere;
		this.radius = radius;
	}

	public function toString() {
		return "Sphere";
	}

	public inline function getLocalBounds() {
		return new AABB(new Vec3(-radius, -radius, -radius), new Vec3(radius, radius, radius));
	}

	public inline function getLocalBoundsToBuffer( out : AABB ) {
		out.load(getLocalBounds());
	}

	public inline function getSurfaceNormal( localPos : Vec3 ) : Vec3 {
		return localPos.normalized();
	}

	public inline function raycast( ray : Ray, scale : Vec3, transform : Mat, infos : HitResult ) : Bool {
		var spherePos = transform.getPosition();
		var torigin = ray.origin - spherePos;
		var tdirection = ray.direction;
		var fraction = Math.localRaySphere(torigin, tdirection, radius * scale.x);
		if ( fraction < Math.SCALAR_MAX ) {
			infos.position.load(ray.getPoint(fraction));
			infos.normal.load(getSurfaceNormal(infos.position - spherePos));
			infos.fraction = fraction;
			return true;
		}
		return false;
	}

	public inline function isScaleValid( scale : Vec3 ) {
		return !ScaleHelper.isNearZero(scale) && ScaleHelper.isUniform(scale);
	}

	public inline function makeScaleValid( scale : Vec3 ) {
		scale.load(ScaleHelper.makeNonZero(scale));
		scale.load(ScaleHelper.makeUniform(scale));
	}

	public inline function getSupportClass() {
		return SphereSupport;
	}

	#if heaps
	public static function fromHeaps( sphere : h3d.col.Sphere, position : Vec3, rotation : Vec3 ) : SphereShape {
		var shape = new SphereShape(sphere.r);
		position.set(sphere.x, sphere.y, sphere.z);
		rotation.set(0.0, 0.0, 0.0);
		return shape;
	}
	#end
}
