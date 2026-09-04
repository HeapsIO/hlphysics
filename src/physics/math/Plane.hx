package physics.math;

@:struct
class Plane {
	public var nx : Scalar;
	public var ny : Scalar;
	public var nz : Scalar;
	public var distance : Scalar;

	public inline function new( nx : Scalar = 0.0, ny : Scalar = 0.0, nz : Scalar = 0.0, distance : Scalar = 0.0 ) {
		set(nx, ny, nz, distance);
	}

	public inline function set( nx : Scalar, ny : Scalar, nz : Scalar, distance : Scalar ) {
		this.nx = nx;
		this.ny = ny;
		this.nz = nz;
		this.distance = distance;
	}

	public inline function setFromPoints( p0 : Vec3, p1 : Vec3, p2 : Vec3, insidePoint : Vec3 ) {
		var normal = (p1 - p0).cross(p2 - p0);
		Assert.t(normal.lengthSq() > 0.0);
		var distance = normal.dot(p0);
		if( normal.dot(insidePoint) - distance < 0.0 ) {
			normal.scale(-1.0);
			distance = -distance;
		}
		set(normal.x, normal.y, normal.z, distance);
	}
}
