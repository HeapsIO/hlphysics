package shiro.physics.collision.shapes;

class ScaledShape extends Shape {

	public var shape : Shape;
	@:packed public var scale : Vec3;

	@:packed var tmpVec : Vec3;

	public inline function new( shape : Shape, scale : Vec3 ) {
		shapeType = Scaled;
		this.shape = shape;
		this.scale = scale;
	}

	public function toString() {
		return "Scaled(" + shape.toString() + ")";
	}

	public inline function getLocalBounds() {
		return shape.getLocalBounds().scaled(scale);
	}

	public inline function getLocalBoundsToBuffer( out : AABB ) {
		shape.getLocalBoundsToBuffer(out);
		out.scale(scale);
	}

	public inline function getSurfaceNormal( localPos : Vec3 ) : Vec3 {
		tmpVec.load(localPos.divided(scale));
		var normal = shape.getSurfaceNormal(tmpVec);
		return normal.divided(scale).normalized();
	}

	public function raycast( ray : Ray, scale : Vec3, transform : Mat, infos : HitResult ) : Bool {
		throw "Should not be called directly"; // See ScaledAlgorithm.raycast
	}

	public inline function isScaleValid( scale : Vec3 ) : Bool {
		tmpVec.load(scale.multiplied(this.scale));
		return shape.isScaleValid(tmpVec);
	}

	public inline function makeScaleValid( scale : Vec3 ) {
		tmpVec.load(scale.multiplied(this.scale));
		shape.makeScaleValid(tmpVec);
		scale.load(tmpVec.divided(this.scale));
	}
}
