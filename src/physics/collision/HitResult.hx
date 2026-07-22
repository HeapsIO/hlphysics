package physics.collision;

@:struct
class HitResult {
	@:packed public var position : Vec3;
	@:packed public var normal : Vec3;
	public var fraction : Scalar;

	public inline function new() {
		position = Vec3.zero();
		normal = Vec3.zero();
		fraction = 0.0;
	}

	public inline function clone() {
		var c = new HitResult();
		c.load(this);
		return c;
	}

	public inline function load( v : HitResult ) {
		position.load(v.position);
		normal.load(v.normal);
		fraction = v.fraction;
	}
}
