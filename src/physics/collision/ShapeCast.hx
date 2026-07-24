package physics.collision;

@:struct
class ShapeCast {
	inline static var DEFAULT_COLLISION_TOLERANCE = 1.0e-4;

	public var shape : Shape;
	/**
		Should only contains `position` and `rotation`.
	**/
	@:packed public var transform(default, set) : Mat;
	@:packed public var direction(default, set) : Vec3;
	@:packed public var scale(default, set) : Vec3;
	public var collisionTolerance : Scalar;

	public function new() {
		transform.load(Mat.identity());
		scale.load(Vec3.one());
		collisionTolerance = DEFAULT_COLLISION_TOLERANCE;
	}

	inline function set_transform( t : Mat ) {
		transform.load(t);
		return transform;
	}

	public inline function setDirection( lx : Scalar, ly : Scalar, lz : Scalar ) {
		direction.x = lx;
		direction.y = ly;
		direction.z = lz;
	}

	inline function set_direction( dir : Vec3 ) {
		direction.load(dir);
		return direction;
	}

	public inline function setScale( sx : Scalar, sy : Scalar, sz : Scalar ) {
		scale.x = sx;
		scale.y = sy;
		scale.z = sz;
		Assert.t(shape.isScaleValid(scale));
	}

	inline function set_scale( s : Vec3 ) {
		scale.load(s);
		Assert.t(shape.isScaleValid(scale));
		return scale;
	}

	public inline function getPoint( fraction : Scalar ) : Vec3 {
		return transform.getPosition() + direction * fraction;
	}
}
