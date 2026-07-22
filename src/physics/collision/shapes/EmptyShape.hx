package shiro.physics.collision.shapes;

class EmptyShape extends Shape {

	public inline function new() {
		shapeType = Empty;
	}

	public function toString() {
		return "Empty";
	}

	public inline function getLocalBounds() {
		return AABB.empty();
	}

	public inline function getLocalBoundsToBuffer( out : AABB ) {
		out.load(getLocalBounds());
	}

	public inline function getSurfaceNormal( localPos : Vec3 ) : Vec3 {
		return new Vec3(0.0, 0.0, 1.0);
	}

	public function raycast( ray : Ray, scale : Vec3, transform : Mat, infos : HitResult ) : Bool {
		return false;
	}

	public inline function isScaleValid( scale : Vec3 ) : Bool {
		return true;
	}

	public inline function makeScaleValid( scale : Vec3 ) {
	}
}
