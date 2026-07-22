package shiro.physics.collision.narrowphase;

class CapsuleVsCapsuleAlgorithm {

	public static function testCollision( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) : Bool {
		var capsule1ToCapsule2SpaceTransform = new Mat();
		capsule1ToCapsule2SpaceTransform.multiply3x4inline(transform1, transform2.getInverse());

		var capsuleShape1 : CapsuleShape = hl.Api.unsafeCast(shape1);
		var capsuleShape2 : CapsuleShape = hl.Api.unsafeCast(shape2);

		var capsule1HalfHeight = capsuleShape1.halfHeight * scale1.x;
		var capsule2HalfHeight = capsuleShape2.halfHeight * scale2.x;
		var capsule1Radius = capsuleShape1.radius * scale1.x;
		var capsule2Radius = capsuleShape2.radius * scale2.x;

		var capsule1SegA = new Vec3(0, 0, -capsule1HalfHeight);
		var capsule1SegB = new Vec3(0, 0, capsule1HalfHeight);
		capsule1SegA.load(capsule1SegA * capsule1ToCapsule2SpaceTransform);
		capsule1SegB.load(capsule1SegB * capsule1ToCapsule2SpaceTransform);

		var capsule2SegA = new Vec3(0, 0, -capsule2HalfHeight);
		var capsule2SegB = new Vec3(0, 0, capsule2HalfHeight);

		var seg1 = capsule1SegB - capsule1SegA;
		var seg2 = capsule2SegB - capsule2SegA;

		var sumRadius = capsule1Radius + capsule2Radius;

		var areCapsuleInnerSegmentsParralel = seg1.cross(seg2).lengthSq() < 0.00001;
		if (areCapsuleInnerSegmentsParralel) {
			var segmentsPerpendicularDistance = Math.computePointToLineDistance(capsule1SegA, capsule1SegB, capsule2SegA);
			if (segmentsPerpendicularDistance >= sumRadius)
				return false;

			var d1 = seg1.dot(capsule1SegA);
			var d2 = -seg1.dot(capsule1SegB);

			var t1 = Math.computePlaneSegmentIntersection(capsule2SegB, capsule2SegA, d1, seg1);
			var t2 = Math.computePlaneSegmentIntersection(capsule2SegA, capsule2SegB, d2, seg1.scaled(-1));

			if (t1 > 0.0 && t2 > 0.0) {
				if (t1 > 1.0)
					t1 = 1.0;
				var clipPointA = capsule2SegB - t1 * seg2;
				if (t2 > 1.0)
					t2 = 1.0;
				var clipPointB = capsule2SegA + t2 * seg2;

				var seg1Normalized = seg1.normalized();
				var pointOnInnerSegCapsule1 = capsule1SegA + seg1Normalized.dot(capsule2SegA - capsule1SegA) * seg1Normalized;

				var normalCapsule2SpaceNormalized = new Vec3();
				var segment1ToSegment2 = new Vec3();

				if (segmentsPerpendicularDistance > Math.EPSILON) {
					segment1ToSegment2.load((capsule2SegA - pointOnInnerSegCapsule1));
					normalCapsule2SpaceNormalized.load(segment1ToSegment2.normalized());
				} else {
					var vec1 = new Vec3(1, 0, 0);
					var vec2 = new Vec3(0, 1, 0);

					var seg2Normalized = seg2.normalized();

					var cosA1 = Math.abs(seg2Normalized.x);
					var cosA2 = Math.abs(seg2Normalized.y);

					if ( cosA1 < cosA2 )
						normalCapsule2SpaceNormalized.load(seg2Normalized.cross(vec1));
					else
						normalCapsule2SpaceNormalized.load(seg2Normalized.cross(vec2));
				}

				var capsule2ToCapsule1SpaceTransform = capsule1ToCapsule2SpaceTransform.getInverse();
				var contactPointACapsule1Local = clipPointA - segment1ToSegment2 + normalCapsule2SpaceNormalized * capsule1Radius;
				var contactPointBCapsule1Local = clipPointB - segment1ToSegment2 + normalCapsule2SpaceNormalized * capsule1Radius;
				var contactPointACapsule2Local = clipPointA - normalCapsule2SpaceNormalized * capsule2Radius;
				var contactPointBCapsule2Local = clipPointB - normalCapsule2SpaceNormalized * capsule2Radius;

				var penetration = sumRadius - segmentsPerpendicularDistance;

				var normalWorld = normalCapsule2SpaceNormalized.transformed3x3(transform2);

				collector.addHit(contactPointACapsule1Local.transformed(transform2), contactPointACapsule2Local.transformed(transform2), normalWorld, penetration);
				collector.addHit(contactPointBCapsule1Local.transformed(transform2), contactPointBCapsule2Local.transformed(transform2), normalWorld, penetration);
				return true;
			}
		}

		var closestPointCapsule1Seg = new Vec3();
		var closestPointCapsule2Seg = new Vec3();
		Math.computeClosestPointBetweenTwoSegments(capsule1SegA, capsule1SegB, capsule2SegA, capsule2SegB, closestPointCapsule1Seg, closestPointCapsule2Seg);

		var closestPointsSeg1ToSeg2 = (closestPointCapsule2Seg - closestPointCapsule1Seg);
		var closestPointsDistanceSquare = closestPointsSeg1ToSeg2.lengthSq();

		if (closestPointsDistanceSquare < sumRadius * sumRadius) {
			if (closestPointsDistanceSquare > Math.EPSILON) {
				var closestPointsDistance = Math.sqrt(closestPointsDistanceSquare);
				closestPointsSeg1ToSeg2.scale(1.0 / closestPointsDistance);

				var contactPointCapsule1Local = closestPointCapsule1Seg + closestPointsSeg1ToSeg2 * capsule1Radius;
				var contactPointCapsule2Local = closestPointCapsule2Seg - closestPointsSeg1ToSeg2 * capsule2Radius;
				var normalWorld = closestPointsSeg1ToSeg2.transformed3x3(transform2);
				var penetration = Math.max(sumRadius - closestPointsDistance, Math.EPSILON);

				collector.addHit(contactPointCapsule1Local.transformed(transform2), contactPointCapsule2Local.transformed(transform2), normalWorld, penetration);
			} else if (areCapsuleInnerSegmentsParralel) {
				var squareDistCapsule2PointToCapsuleSegA = (capsule1SegA - closestPointCapsule2Seg).lengthSq();

				var capsule1SegmentMostExtremePoint = new Vec3();
				if ( squareDistCapsule2PointToCapsuleSegA > Math.EPSILON )
					capsule1SegmentMostExtremePoint.load(capsule1SegA);
				else
					capsule1SegmentMostExtremePoint.load(capsule1SegB);
				var normalCapsuleSpace2 = (closestPointCapsule2Seg - capsule1SegmentMostExtremePoint);
				normalCapsuleSpace2.normalize();

				var contactPointCapsule1Local = closestPointCapsule1Seg + normalCapsuleSpace2 * capsule1Radius;
				var contactPointCapsule2Local = closestPointCapsule2Seg - normalCapsuleSpace2 * capsule2Radius;
				var normalWorld = normalCapsuleSpace2.transformed3x3(transform2);

				collector.addHit(contactPointCapsule1Local.transformed(transform2), contactPointCapsule2Local.transformed(transform2), normalWorld, sumRadius);
			} else {
				var normalCapsuleSpace2 = seg1.cross(seg2);
				normalCapsuleSpace2.normalize();

				var contactPointCapsule1Local = closestPointCapsule1Seg + normalCapsuleSpace2 * capsule1Radius;
				var contactPointCapsule2Local = closestPointCapsule2Seg - normalCapsuleSpace2 * capsule2Radius;
				var normalWorld = normalCapsuleSpace2.transformed3x3(transform2);

				collector.addHit(contactPointCapsule1Local.transformed(transform2), contactPointCapsule2Local.transformed(transform2), normalWorld, sumRadius);
			}

			return true;
		}

		return false;
	}
}
