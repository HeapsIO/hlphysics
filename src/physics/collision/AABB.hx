package physics.collision;

enum abstract PlaneIntersection(Int) {
	var Back = -1;
	var Intersecting = 0;
	var Front = 1;
}

@:struct
class AABB {
	@:packed public var min : Vec3;
	@:packed public var max : Vec3;

	public static inline function empty() {
		return new AABB(new Vec3(0.0, 0.0, 0.0), new Vec3(0.0, 0.0, 0.0));
	}

	public inline function new( min : Vec3, max : Vec3 ) {
		this.min = min;
		this.max = max;
	}

	public inline function load( b : AABB ) {
		min.load(b.min);
		max.load(b.max);
	}

	public inline function collide( b : AABB ) : Bool {
		return !(min.x > b.max.x || min.y > b.max.y || min.z > b.max.z || max.x < b.min.x || max.y < b.min.y || max.z < b.min.z);
	}

	public inline function containsAABB( b : AABB ) {
		return min.x <= b.min.x && min.y <= b.min.y && min.z <= b.min.z && max.x >= b.max.x && max.y >= b.max.y && max.z >= b.max.z;
	}

	public inline function intersectPlane( plane : Plane ) : PlaneIntersection {
		var nx = plane.nx;
		var ny = plane.ny;
		var nz = plane.nz;
		var centerDistance2 = nx * (max.x + min.x) + ny * (max.y + min.y) + nz * (max.z + min.z) - 2.0 * plane.distance;
		var radius2 = Math.abs(nx) * (max.x - min.x) + Math.abs(ny) * (max.y - min.y) + Math.abs(nz) * (max.z - min.z);
		if( centerDistance2 + radius2 < 0.0 )
			return Back;
		if( centerDistance2 - radius2 >= 0.0 )
			return Front;
		return Intersecting;
	}

	public inline function raycast( ray : Ray ) : Scalar {
		return Math.localRayAABB(ray.origin, ray.direction, min, max);
	}

	public inline function getCenter() : Vec3 {
		return 0.5 * (min + max);
	}

	public inline function getExtent() : Vec3 {
		return max - min;
	}

	public inline function getVolume() : Scalar {
		var extent = getExtent();
		return extent.x * extent.y * extent.z;
	}

	public inline function offset( v : Vec3 ) : AABB {
		min += v;
		max += v;
		return this;
	}

	public inline function clone() : AABB {
		return new AABB(min.clone(), max.clone());
	}

	public inline function merge( a : AABB ) {
		min.load(Vec3.min(min, a.min));
		max.load(Vec3.max(max, a.max));
	}

	public inline function mergeTwo( a : AABB, b : AABB ) {
		min.load(Vec3.min(a.min, b.min));
		max.load(Vec3.max(a.max, b.max));
	}

	public inline function addPos( pos : Vec3 ) {
		min.load(Vec3.min(min, pos));
		max.load(Vec3.max(max, pos));
	}

	public inline function enlargeWithExtent( extent : Vec3 ) {
		min.load(min - extent);
		max.load(max + extent);
	}

	public inline function scale( v : Vec3 ) {
		min *= v;
		max *= v;
	}

	public inline function scaled( v : Vec3 ) : AABB {
		var c = this.clone();
		c.scale(v);
		return c;
	}

	public inline function transform( m : Mat ) {
		var oldMin = min.clone();
		var oldMax = max.clone();
		var offset = m.getPosition();
		min.load(offset);
		max.load(offset);

		var col = new Vec3(m._11, m._12, m._13);
		var a = col * oldMin.x;
		var b = col * oldMax.x;

		min += Vec3.min(a, b);
		max += Vec3.max(a, b);

		col.set(m._21, m._22, m._23);
		var a = col * oldMin.y;
		var b = col * oldMax.y;

		min += Vec3.min(a, b);
		max += Vec3.max(a, b);

		col.set(m._31, m._32, m._33);
		var a = col * oldMin.z;
		var b = col * oldMax.z;

		min += Vec3.min(a, b);
		max += Vec3.max(a, b);
	}

	public inline function transformed( m : Mat ) : AABB {
		var c = this.clone();
		c.transform(m);
		return c;
	}

	public function toString() {
		return '{min:$min,max:$max}';
	}
}
