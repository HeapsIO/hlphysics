package physics.math;

class ScaleHelper {
	public static inline var SCALE_MIN : Scalar = 1.0e-6;
	public static inline var SCALE_TOLERANCE_SQ : Scalar = 1.0e-8;

	public static inline function isNotScaled( scale : Vec3 ) : Bool {
		return scale.isClose(Vec3.one(), SCALE_TOLERANCE_SQ);
	}

	public static inline function isUniform( scale : Vec3 ) : Bool {
		var s2 = new Vec3(scale.y, scale.z, scale.x);
		return scale.isClose(s2, SCALE_TOLERANCE_SQ);
	}

	public static inline function isUniformXY( scale : Vec3 ) : Bool {
		var s2 = new Vec3(scale.y, scale.x, scale.z);
		return scale.isClose(s2, SCALE_TOLERANCE_SQ);
	}

	public static inline function isNearZero( scale : Vec3 ) : Bool {
		var abs = scale.abs();
		return abs.x < SCALE_MIN || abs.y < SCALE_MIN || abs.z < SCALE_MIN;
	}

	public static inline function makeUniform( scale : Vec3 ) : Vec3 {
		var avg = (scale.x + scale.y + scale.z) / 3.0;
		return new Vec3(avg, avg, avg);
	}

	public static inline function makeUniformXY( scale : Vec3 ) : Vec3 {
		var avg = (scale.x + scale.y) * 0.5;
		return new Vec3(avg, avg, scale.z);
	}

	public static inline function makeNonZero( scale : Vec3 ) : Vec3 {
		var sx = Math.max(Math.abs(scale.x), SCALE_MIN);
		if( scale.x < 0 ) sx = -sx;
		var sy = Math.max(Math.abs(scale.y), SCALE_MIN);
		if( scale.y < 0 ) sy = -sy;
		var sz = Math.max(Math.abs(scale.z), SCALE_MIN);
		if( scale.z < 0 ) sz = -sz;
		return new Vec3(sx, sy, sz);
	}

	public static inline function canScaleBeRotated( rotation : Quat, scale : Vec3 ) : Bool {
		var rmat = new Mat();
		rmat.initRotationQuat(rotation);
		var childScale = rmat.scaled(scale.x, scale.y, scale.z).multiplied(rmat.transposed());

		var epsilon = 10e-6;
		return Math.abs(childScale._12) < epsilon && Math.abs(childScale._13) < epsilon && Math.abs(childScale._14) < epsilon
			&& Math.abs(childScale._21) < epsilon && Math.abs(childScale._23) < epsilon && Math.abs(childScale._24) < epsilon
			&& Math.abs(childScale._31) < epsilon && Math.abs(childScale._32) < epsilon && Math.abs(childScale._34) < epsilon;
	}

	public static inline function rotateScale( rotation : Quat, scale : Vec3 ) : Vec3 {
		var rmat = new Mat();
		rmat.initRotationQuat(rotation);
		var childScale = rmat.scaled(scale.x, scale.y, scale.z).multiplied(rmat.transposed());
		return new Vec3(childScale._11, childScale._22, childScale._33);
	}
}
