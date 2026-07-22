package shiro.physics.math;

@:struct
class Quat {
	public static inline function identity() {
		return new Quat(0.0, 0.0, 0.0, 1.0);
	}

	public static inline function identityNeg() {
		return new Quat(0.0, 0.0, 0.0, -1.0);
	}

	@:packed public var value : Vec4;

	public var x(get, set) : Scalar;
	public inline function get_x() : Scalar {
		return value.x;
	}
	public inline function set_x( v : Scalar ) {
		return value.x = v;
	}

	public var y(get, set) : Scalar;
	public inline function get_y() : Scalar {
		return value.y;
	}
	public inline function set_y( v : Scalar ) {
		return value.y = v;
	}

	public var z(get, set) : Scalar;
	public inline function get_z() : Scalar {
		return value.z;
	}
	public inline function set_z( v : Scalar ) {
		return value.z = v;
	}

	public var w(get, set) : Scalar;
	public inline function get_w() : Scalar {
		return value.w;
	}
	public inline function set_w( v : Scalar ) {
		return value.w = v;
	}

	public inline function new( x : Scalar = 0.0, y : Scalar = 0.0, z : Scalar = 0.0, w : Scalar = 0.0 ) {
		value = new Vec4(x, y, z, w);
	}

	public inline function load( v : Quat ) {
		value.load(v.value);
	}

	public inline function set( x : Scalar, y : Scalar, z : Scalar, w : Scalar ) {
		this.value.x = x;
		this.value.y = y;
		this.value.z = z;
		this.value.w = w;
	}

	public inline function toMatrix() {
		var m = new Mat();
		m.initRotationQuat(this);
		return m;
	}

	public inline function normalize() {
		var lenSq = x * x + y * y + z * z + w * w;
		if( lenSq < Math.EPSILON ) {
			x = y = z = 0;
			w = 1;
		} else {
			var m = 1. / Math.sqrt(lenSq);
			x = x * m;
			y = y * m;
			z = z * m;
			w = w * m;
		}
	}

	public inline function initRotation( ax : Float, ay : Float, az : Float ) {
		var sinX = Math.sin( ax * 0.5 );
		var cosX = Math.cos( ax * 0.5 );
		var sinY = Math.sin( ay * 0.5 );
		var cosY = Math.cos( ay * 0.5 );
		var sinZ = Math.sin( az * 0.5 );
		var cosZ = Math.cos( az * 0.5 );
		var cosYZ = cosY * cosZ;
		var sinYZ = sinY * sinZ;
		x = sinX * cosYZ - cosX * sinYZ;
		y = cosX * sinY * cosZ + sinX * cosY * sinZ;
		z = cosX * cosY * sinZ - sinX * sinY * cosZ;
		w = cosX * cosYZ + sinX * sinYZ;
	}

	public static inline function fromTo( from : Vec3, to : Vec3 ) : Quat {
		var len = Math.sqrt(from.lengthSq() * to.lengthSq());
		var w = len + from.dot(to);
		var q = new Quat();
		if( w == 0.0 ) {
			if( len == 0.0 ) {
				q.load(Quat.identity());
			} else {
				var per = from.getNormalizedPerpendicular();
				q.set(per.x, per.y, per.z, w);
			}
		} else {
			var v = from.cross(to);
			q.set(v.x, v.y, v.z, w);
			q.normalize();
		}
		return q;
	}

	public inline function getEulerAngles() : Vec3 {
		var y2 = y * y;

		var t0 = 2.0 * (w * x + y * z);
		var t1 = 1.0 - 2.0 * (x * x + y2);

		var t2 = 2.0 * (w * y - z * x);
		t2 = t2 > 1.0 ? 1.0 : t2;
		t2 = t2 < -1.0 ? -1.0 : t2;

		var t3 = 2.0 * (w * z + x * y);
		var t4 = 1.0 - 2.0 * (y2 + z * z);

		return new Vec3(Math.atan2(t0, t1), Math.asin(t2), Math.atan2(t3, t4));
	}

	public function toString() {
		return value.toString();
	}

	/**
		Note: q and -q represent the same rotation and is not checked here
	**/
	public inline function isClose( v : Quat, maxDistSq : Scalar = 1.0e-12 ) : Bool {
		return (v.value - this.value).lengthSq() <= maxDistSq;
	}

	#if heaps
	public static inline function fromHeaps( v : h3d.Quat ) {
		return new Quat(v.x, v.y, v.z, v.w);
	}

	public inline function toHeaps() : h3d.Quat {
		return new h3d.Quat(x, y, z, w);
	}
	#end

}
