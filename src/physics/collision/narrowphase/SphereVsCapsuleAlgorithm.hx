package shiro.physics.collision.narrowphase;

class SphereVsCapsuleAlgorithm {

	static function collideSphereVsCapsule( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat ) : Null<ContactPoint> {

		var sphereShape : SphereShape = hl.Api.unsafeCast(shape1);
		var capsuleShape : CapsuleShape = hl.Api.unsafeCast(shape2);

		var capsuleHalfHeight = capsuleShape.halfHeight * scale2.x;
		var sphereRadius = sphereShape.radius * scale1.x;
		var capsuleRadius = capsuleShape.radius * scale2.x;

		var sphereToWorldTransform = transform1;
		var capsuleToWorldTransform = transform2;
		var worldToCapsuleTransform = capsuleToWorldTransform.getInverse();
		var sphereToCapsuleSpaceTransform = sphereToWorldTransform * worldToCapsuleTransform;

		var sphereCenter = sphereToCapsuleSpaceTransform.getPosition();

		var capsuleSegA = new Vec3(0, 0, -capsuleHalfHeight);
		var capsuleSegB = new Vec3(0, 0, capsuleHalfHeight);

		var closestPointOnSegment = Math.computeClosestPointOnSegment(capsuleSegA, capsuleSegB, sphereCenter);

		var sphereCenterToSegment = (closestPointOnSegment - sphereCenter);
		var sphereSegmentDistanceSquare = sphereCenterToSegment.lengthSq();

		var sumRadius = sphereRadius + capsuleRadius;

		if (sphereSegmentDistanceSquare < sumRadius * sumRadius) {

			var penetration : Scalar;
			var normalWorld = new Vec3();
			var contactPointOnSphere = new Vec3();
			var contactPointOnCapsule = new Vec3();

			if (sphereSegmentDistanceSquare > Math.EPSILON) {
				var sphereSegmentDistance = Math.sqrt(sphereSegmentDistanceSquare);
				sphereCenterToSegment.scale(1.0 / sphereSegmentDistance);

				penetration = sumRadius - sphereSegmentDistance;
				if (penetration <= 0.0)
					return null;

				contactPointOnSphere.load((sphereCenter + sphereCenterToSegment * sphereRadius).transformed(capsuleToWorldTransform));
				contactPointOnCapsule.load((closestPointOnSegment - sphereCenterToSegment * capsuleRadius).transformed(capsuleToWorldTransform));

				normalWorld.load(sphereCenterToSegment.transformed3x3(capsuleToWorldTransform));

			} else {
				var capsuleSegment = (capsuleSegB - capsuleSegA).normalized();

				var vec1 = new Vec3(1, 0, 0);
				var vec2 = new Vec3(0, 1, 0);

				var cosA1 = Math.abs(capsuleSegment.x);
				var cosA2 = Math.abs(capsuleSegment.y);

				penetration = sumRadius;

				if (penetration <= 0.0)
					return null;

				var normalCapsuleSpace = new Vec3();
				if (cosA1 < cosA2)
					normalCapsuleSpace.load(capsuleSegment.cross(vec1))
				else
					normalCapsuleSpace.load(capsuleSegment.cross(vec2));
				normalWorld.load(normalCapsuleSpace.transformed3x3(capsuleToWorldTransform));

				contactPointOnSphere.load((sphereCenter + normalCapsuleSpace * sphereRadius).transformed(capsuleToWorldTransform));
				contactPointOnCapsule.load((sphereCenter - normalCapsuleSpace * capsuleRadius).transformed(capsuleToWorldTransform));
			}

			var result = new ContactPoint();
			result.contactPointOn1.load(contactPointOnSphere);
			result.contactPointOn2.load(contactPointOnCapsule);
			result.normal.load(normalWorld);
			result.penetration = penetration;
			return result;
		}

		return null;
	}

	public static function testCollision( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) {
		var r = collideSphereVsCapsule(shape1, shape2, scale1, scale2, transform1, transform2);
		if ( r != null ) {
			collector.addHit(r.contactPointOn1, r.contactPointOn2, r.normal, r.penetration);
			return true;
		}
		return false;
	}

	public static function testCollision2( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) {
		var r = collideSphereVsCapsule(shape2, shape1, scale2, scale1, transform2, transform1);
		if ( r != null ) {
			collector.addHit(r.contactPointOn2, r.contactPointOn1, r.normal.scaled(-1.0), r.penetration);
			return true;
		}
		return false;
	}
}
