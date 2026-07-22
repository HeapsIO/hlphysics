package physics.collision;

@:struct
class Ray {
	@:packed public var origin(default, set) : Vec3;
	@:packed public var direction(default, set) : Vec3;

	public function new() {
	}

	public inline function clone() : Ray {
		var ray = new Ray();
		ray.load(this);
		return ray;
	}

	public inline function load( r : Ray ) {
		origin.load(r.origin);
		direction.load(r.direction);
	}

	inline public function setOrigin( x : Scalar, y : Scalar, z : Scalar ) {
		origin.x = x;
		origin.y = y;
		origin.z = z;
	}

	inline function set_origin( o : Vec3 ) {
		origin.load(o);
		return origin;
	}

	inline public function setDirection( lx : Scalar, ly : Scalar, lz : Scalar ) {
		direction.x = lx;
		direction.y = ly;
		direction.z = lz;
	}

	inline function set_direction( dir : Vec3 ) {
		direction.load(dir);
		return direction;
	}

	public inline function getPoint( fraction : Scalar ) : Vec3 {
		return origin + direction * fraction;
	}

	#if heaps
	public static inline function fromHeaps( v : h3d.col.Ray ) : Ray {
		var r = new Ray();
		var pos = v.getPos();
		var dir = v.getDir();
		r.setOrigin(pos.x, pos.y, pos.z);
		r.setDirection(dir.x, dir.y, dir.z);
		return r;
	}

	public inline function toHeaps() :h3d.col.Ray {
		var ray = new h3d.col.Ray();
		ray.px = origin.x;
		ray.py = origin.y;
		ray.pz = origin.z;
		ray.lx = origin.x;
		ray.ly = origin.y;
		ray.lz = origin.z;
		return ray;
	}
	#end
}
