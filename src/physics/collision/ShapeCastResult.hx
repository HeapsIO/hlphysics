package shiro.physics.collision;

@:struct
class ShapeCastResult {
	/**
		Contact point on the surface of shape 1 (in world space)
	**/
	@:packed public var contactPointOn1 : Vec3;
	/**
		Contact point on the surface of shape 2 (in world space)
	**/
	@:packed public var contactPointOn2 : Vec3;
	@:packed public var penetrationAxis : Vec3;
	public var penetration : Scalar;
	public var fraction : Scalar;

	public inline function new() {
		contactPointOn1 = Vec3.zero();
		contactPointOn2 = Vec3.zero();
		penetrationAxis = Vec3.zero();
		penetration = 0.0;
		fraction = 0.0;
	}

	public inline function getNormal() {
		return penetrationAxis.normalized().scaled(-1.0);
	}

	public inline function load( s : ShapeCastResult ) {
		contactPointOn1.load(s.contactPointOn1);
		contactPointOn2.load(s.contactPointOn2);
		penetrationAxis.load(s.penetrationAxis);
		penetration = s.penetration;
		fraction = s.fraction;
	}

	public inline function clone() {
		var c = new ShapeCastResult();
		c.contactPointOn1 = contactPointOn1;
		c.contactPointOn2 = contactPointOn2;
		c.penetrationAxis = penetrationAxis;
		c.penetration = penetration;
		c.fraction = fraction;
		return c;
	}
}
