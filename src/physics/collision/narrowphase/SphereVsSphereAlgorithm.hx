package physics.collision.narrowphase;

class SphereVsSphereAlgorithm {

	public static function testCollision( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) {
		var delta = transform2.getPosition() - transform1.getPosition();
		var lengthSq = delta.lengthSq();

		var shape1 : SphereShape = hl.Api.unsafeCast(shape1);
		var shape2 : SphereShape = hl.Api.unsafeCast(shape2);

		var radius1 : Scalar = shape1.radius * scale1.x;
		var radius2 : Scalar = shape2.radius * scale2.x;

		var sumRadius = radius1 + radius2;
		var sumRadius2 = sumRadius * sumRadius;

		if (lengthSq < sumRadius2) {
			var penetrationDepth = sumRadius - Math.sqrt(lengthSq);
			if ( penetrationDepth > 0 ) {
				var normal = lengthSq > Math.EPSILON ? delta.normalized() : new Vec3(0, 0, 1);
				var p1 = (radius1 * normal).added(transform1.getPosition());
				var p2 = (-radius2 * normal).added(transform2.getPosition());
				collector.addHit(p1, p2, normal, penetrationDepth);
				return true;
			}
		}
		return false;
	}
}
