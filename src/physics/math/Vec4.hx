package shiro.physics.math;

@:struct
class Vec4Impl {
	public var x : Scalar;
	public var y : Scalar;
	public var z : Scalar;
	public var w : Scalar;

	public inline function new( x : Scalar, y : Scalar, z : Scalar, w : Scalar ) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
	}

	public inline function load( v : Vec4 ) {
		x = v.x;
		y = v.y;
		z = v.z;
		w = v.w;
	}

	public inline function set( x : Scalar, y : Scalar, z : Scalar, w : Scalar ) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
	}

	public inline function sub( v : Vec4 ) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
		w -= w;
	}

	public inline function subbed( v : Vec4 ) {
		return new Vec4(x - v.x, y - v.y, z - v.z, w - v.w);
	}

	public inline function add( v : Vec4 ) {
		x += v.x;
		y += v.y;
		z += v.z;
		w += v.w;
	}

	public inline function added( v : Vec4 ) {
		return new Vec4(x + v.x, y + v.y, z + v.z, w + v.w);
	}

	public inline function scale( f : Scalar ) {
		x *= f;
		y *= f;
		z *= f;
		w *= f;
	}

	public inline function scaled( f : Scalar ) : Vec4 {
		return new Vec4(x * f, y * f, z * f, w * f);
	}

	public inline function divide( v : Vec4 ) {
		x /= v.x;
		y /= v.y;
		z /= v.z;
		w /= v.w;
	}

	public inline function divided( v : Vec4 ) : Vec4 {
		return new Vec4(x / v.x, y / v.y, z / v.z, w / v.w);
	}

	public inline function transform( m : Mat ) {
		var px = x * m._11 + y * m._21 + z * m._31 + w * m._41;
		var py = x * m._12 + y * m._22 + z * m._32 + w * m._42;
		var pz = x * m._13 + y * m._23 + z * m._33 + w * m._43;
		var pw = x * m._14 + y * m._24 + z * m._34 + w * m._44;
		x = px;
		y = py;
		z = pz;
		w = pw;
	}

	public inline function transformed( m : Mat ) {
		var px = x * m._11 + y * m._21 + z * m._31 + w * m._41;
		var py = x * m._12 + y * m._22 + z * m._32 + w * m._42;
		var pz = x * m._13 + y * m._23 + z * m._33 + w * m._43;
		var pw = x * m._14 + y * m._24 + z * m._34 + w * m._44;
		return new Vec4(px,py,pz,pw);
	}

	public inline function transform3x4( m : Mat ) {
		var px = x * m._11 + y * m._21 + z * m._31 + w * m._41;
		var py = x * m._12 + y * m._22 + z * m._32 + w * m._42;
		var pz = x * m._13 + y * m._23 + z * m._33 + w * m._43;
		x = px;
		y = py;
		z = pz;
	}

	public inline function transformed3x4( m : Mat ) {
		var px = x * m._11 + y * m._21 + z * m._31 + w * m._41;
		var py = x * m._12 + y * m._22 + z * m._32 + w * m._42;
		var pz = x * m._13 + y * m._23 + z * m._33 + w * m._43;
		return new Vec4(px,py,pz);
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
		return new Vec4(px,py,pz);
	}

	public function toString() {
		return '{$x,$y,$z,$w}';
	}

	public inline function lengthSq() : Scalar {
		return x * x + y * y + z * z + w * w;
	}
}

@:forward abstract Vec4(Vec4Impl) from Vec4Impl to Vec4Impl {
	public inline function new ( x : Scalar = 0.0, y : Scalar = 0.0, z : Scalar = 0.0, w : Scalar = 0.0 ) {
		this = new Vec4Impl(x, y, z, w);
	}

	@:op(a -= b) public inline function sub(v:Vec4) this.sub(v);
	@:op(a - b) public inline function subbed(v:Vec4) return this.subbed(v);
	@:op(a += b) public inline function add(v:Vec4) this.add(v);
	@:op(a + b) public inline function added(v:Vec4) return this.added(v);

	@:op(a *= b) public inline function scale(v:Scalar) this.scale(v);
	@:op(a * b) public inline function scaled(v:Scalar) return this.scaled(v);
	@:op(a * b) static inline function scaledInv( f : Scalar, v : Vec4 ) return v.scaled(f);

	@:op(a /= b) public inline function divide(v:Vec4) this.divide(v);
	@:op(a / b) public inline function divided(v:Vec4) return this.divided(v);

	public static inline function min( v1 : Vec4, v2 : Vec4 ) {
		return new Vec4( Math.min(v1.x, v2.x), Math.min(v1.y, v2.y), Math.min(v1.z, v2.z));
	}

	public static inline function max( v1 : Vec4, v2 : Vec4 ) {
		return new Vec4( Math.max(v1.x, v2.x), Math.max(v1.y, v2.y), Math.max(v1.z, v2.z));
	}

	#if heaps
	public static inline function fromHeaps( v : h3d.Vector4 ) {
		return new Vec4(v.x, v.y, v.z, v.w);
	}

	public inline function toHeaps() : h3d.Vector4 {
		return new h3d.Vector4(this.x, this.y, this.z, this.w);
	}
	#end
}
