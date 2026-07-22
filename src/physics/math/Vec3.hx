package physics.math;

@:struct
class Vec3Impl {
	public var x : Scalar;
	public var y : Scalar;
	public var z : Scalar;

	public inline function new( x : Scalar, y : Scalar, z : Scalar ) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public inline function load( v : Vec3 ) {
		x = v.x;
		y = v.y;
		z = v.z;
	}

	public inline function set( x : Scalar, y : Scalar, z : Scalar ) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public inline function clone() {
		return new Vec3(x, y, z);
	}

	public inline function sub( v : Vec3 ) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
	}

	public inline function subbed( v : Vec3 ) {
		return new Vec3(x - v.x, y - v.y, z - v.z);
	}

	public inline function add( v : Vec3 ) {
		x += v.x;
		y += v.y;
		z += v.z;
	}

	public inline function added( v : Vec3 ) {
		return new Vec3(x + v.x, y + v.y, z + v.z);
	}

	public inline function scale( f : Scalar ) {
		x *= f;
		y *= f;
		z *= f;
	}

	public inline function scaled( f : Scalar ) : Vec3 {
		return new Vec3( x * f, y * f, z * f);
	}

	public inline function multiply( v : Vec3 ) {
		x *= v.x;
		y *= v.y;
		z *= v.z;
	}

	public inline function multiplied( v : Vec3 ) : Vec3 {
		return new Vec3( x * v.x, y * v.y, z * v.z);
	}

	public inline function divide( v : Vec3 ) {
		x /= v.x;
		y /= v.y;
		z /= v.z;
	}

	public inline function divided( v : Vec3 ) : Vec3 {
		return new Vec3( x / v.x, y / v.y, z / v.z);
	}

	public inline function normalize() {
		this.scale(1.0 / length());
	}

	public inline function normalized() : Vec3 {
		return this.scaled(1.0 / length());
	}

	public inline function normalizedOr( alternate : Vec3 ) : Vec3 {
		var v = new Vec3();
		var lenSq = lengthSq();
		if ( lenSq <= Math.EPSILON )
			v.load(alternate.clone());
		else
			v.load(this.scaled(1.0 / (Math.sqrt(lenSq))));
		return v;
	}

	public function toString() {
		return '{$x,$y,$z}';
	}

	public inline function lengthSq() : Scalar {
		return x * x + y * y + z * z;
	}

	public inline function length() : Scalar {
		return Math.sqrt(lengthSq());
	}

	public inline function isClose( v : Vec3, maxDistSq : Scalar ) : Bool {
		return (v - this).lengthSq() <= maxDistSq;
	}

	public inline function isNearZero() : Bool {
		return lengthSq() <= 1.0e-12;
	}

	public inline function hasNaN() : Bool {
		return Math.isNaN(x) || Math.isNaN(y) || Math.isNaN(z);
	}

	public inline function getNormalizedPerpendicular() : Vec3 {
		var result = new Vec3();
		if ( Math.abs(x) > Math.abs(y) ) {
			var invLen = 1.0 / Math.sqrt(x * x + z * z);
			result.load(new Vec3(z, 0.0, -x) * invLen);
		} else {
			var invLen = Math.sqrt(y * y + z * z);
			result.load(new Vec3(0.0, z, -y) * invLen);
		}
		return result;
	}

	public inline function transform( m : Mat ) {
		var px = x * m._11 + y * m._21 + z * m._31 + m._41;
		var py = x * m._12 + y * m._22 + z * m._32 + m._42;
		var pz = x * m._13 + y * m._23 + z * m._33 + m._43;
		x = px;
		y = py;
		z = pz;
	}

	public inline function transformed( m : Mat ) {
		var px = x * m._11 + y * m._21 + z * m._31 + m._41;
		var py = x * m._12 + y * m._22 + z * m._32 + m._42;
		var pz = x * m._13 + y * m._23 + z * m._33 + m._43;
		return new Vec3(px,py,pz);
	}

	public inline function transform3x3( m : Mat ) {
		var px = x * m._11 + y * m._21 + z * m._31;
		var py = x * m._12 + y * m._22 + z * m._32;
		var pz = x * m._13 + y * m._23 + z * m._33;
		x = px;
		y = py;
		z = pz;
	}

	public inline function transformed3x3( m : Mat ) {
		var px = x * m._11 + y * m._21 + z * m._31;
		var py = x * m._12 + y * m._22 + z * m._32;
		var pz = x * m._13 + y * m._23 + z * m._33;
		return new Vec3(px,py,pz);
	}

	public inline function transform3x3Transposed( m : Mat ) {
		var px = x * m._11 + y * m._12 + z * m._13;
		var py = x * m._21 + y * m._22 + z * m._23;
		var pz = x * m._31 + y * m._32 + z * m._33;
		x = px;
		y = py;
		z = pz;
	}

	public inline function transformed3x3Transposed( m : Mat ) {
		var px = x * m._11 + y * m._12 + z * m._13;
		var py = x * m._21 + y * m._22 + z * m._23;
		var pz = x * m._31 + y * m._32 + z * m._33;
		return new Vec3(px,py,pz);
	}

	public inline function dot( b : Vec3 ) : Scalar {
		return x * b.x + y * b.y + z * b.z;
	}

	public inline function cross( b : Vec3 ) : Vec3 {
		return new Vec3( y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x );
	}

	public inline function equals( v : Vec3 ) : Bool {
		return x == v.x && y == v.y && z == v.z;
	}

	public inline function abs() {
		return new Vec3(Math.abs(x), Math.abs(y), Math.abs(z));
	}

	public inline function getLowestComponentIndex() : Int {
		return x < y ? (z < x ? 2 : 0) : (z < y ? 2 : 1);
	}

	public inline function getHighestComponentIndex() : Int {
		return x > y ? (z > x ? 2 : 0) : (z > y ? 2 : 1);
	}
}

@:forward abstract Vec3(Vec3Impl) from Vec3Impl to Vec3Impl {
	public static inline function zero() {
		return new Vec3(0.0, 0.0, 0.0);
	}
	public static inline function one() {
		return new Vec3(1.0, 1.0, 1.0);
	}

	public inline function new ( x : Scalar = 0.0, y : Scalar = 0.0, z : Scalar = 0.0) {
		this = new Vec3Impl(x, y, z);
	}

	@:op(a -= b) public inline function sub(v:Vec3) this.sub(v);
	@:op(a - b) public inline function subbed(v:Vec3) return this.subbed(v);
	@:op(a += b) public inline function add(v:Vec3) this.add(v);
	@:op(a + b) public inline function added(v:Vec3) return this.added(v);

	@:op(a *= b) public inline function scale(v:Scalar) this.scale(v);
	@:op(a * b) public inline function scaled(v:Scalar) return this.scaled(v);
	@:op(a * b) static inline function scaledInv( f : Scalar, v : Vec3 ) return v.scaled(f);

	@:op(a *= b) public inline function multiply(v:Vec3) this.multiply(v);
	@:op(a * b) public inline function multiplied(v:Vec3) return this.multiplied(v);

	@:op(a /= b) public inline function divide(v:Vec3) this.divide(v);
	@:op(a / b) public inline function divided(v:Vec3) return this.divided(v);

	@:op(a *= b) public inline function transform(m:Mat) this.transform(m);
	@:op(a * b) public inline function transformed(m:Mat) return this.transformed(m);

	public static inline function min( v1 : Vec3, v2 : Vec3 ) {
		return new Vec3(Math.min(v1.x, v2.x), Math.min(v1.y, v2.y), Math.min(v1.z, v2.z));
	}

	public static inline function max( v1 : Vec3, v2 : Vec3 ) {
		return new Vec3(Math.max(v1.x, v2.x), Math.max(v1.y, v2.y), Math.max(v1.z, v2.z));
	}

	@:arrayAccess
	public inline function setComponent( i : Int, f : Scalar ) : Scalar {
		var v = this;
		var result : Scalar = switch ( i ) {
			case 0: v.x = f;
			case 1: v.y = f;
			case 2: v.z = f;
			default: 0.0;
		}
		return result;
	}

	@:arrayAccess
	public inline function getComponent( i : Int ) : Scalar {
		var v = this;
		var result : Scalar = switch ( i ) {
			case 0: v.x;
			case 1: v.y;
			case 2: v.z;
			default: 0.0;
		}
		return result;
	}

	#if heaps
	public static inline function fromHeaps( v : h3d.Vector ) : Vec3 {
		return new Vec3(v.x, v.y, v.z);
	}

	public inline function toHeaps() : h3d.Vector {
		return new h3d.Vector(this.x, this.y, this.z);
	}
	#end
}
