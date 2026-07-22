package physics.collision;

class ContactPoint {
	/**
		Contact point on the surface of shape 1 (in world space)
	**/
	@:packed public var contactPointOn1 : Vec3;
	/**
		Contact point on the surface of shape 2 (in world space)
	**/
	@:packed public var contactPointOn2 : Vec3;
	@:packed public var normal : Vec3;
	public var penetration : Scalar;

	public inline function new() {
		contactPointOn1 = Vec3.zero();
		contactPointOn2 = Vec3.zero();
		normal = Vec3.zero();
		penetration = 0.0;
	}

	public inline function load( v : ContactPoint ) {
		contactPointOn1.load(v.contactPointOn1);
		contactPointOn2.load(v.contactPointOn2);
		normal.load(v.normal);
		penetration = v.penetration;
	}
}
