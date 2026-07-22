package shiro.physics.collision.narrowphase;

class EmptyAlgorithm {

	public static function collideShapeVsEmpty( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) : Bool {
		return false;
	}

	public static function collideEmptyVsShape( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) : Bool {
		return false;
	}

	public static function shapecastShapeVsEmpty( shapeCast : ShapeCast, shape2 : Shape, scale2 : Vec3, transform2 : Mat, collector : ShapeCastCollector ) : Bool {
		return false;
	}

	public static function shapecastEmptyVsShape( shapeCast : ShapeCast, shape2 : Shape, scale2 : Vec3, transform2 : Mat, collector : ShapeCastCollector ) : Bool {
		return false;
	}
}
