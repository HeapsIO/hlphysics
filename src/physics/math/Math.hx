package physics.math;

typedef Scalar = #if physics_double_precision Float #else Single #end;

class Math {

	public static inline final SCALAR_MAX : Scalar = 3.40282347E+38;
	public static inline final SCALAR_MIN : Scalar = -3.40282347E+38;
	public static inline final EPSILON : Scalar = 1.192092896E-07;
	public static inline final PI : Scalar = 3.14159265358979323;

	public static inline function cos( a : Scalar ) : Scalar {
		return std.Math.cos(a);
	}

	public static inline function sin( a : Scalar ) : Scalar {
		return std.Math.sin(a);
	}

	public static inline function tan( f : Scalar ) : Scalar {
		return std.Math.tan(f);
	}

	public static inline function acos( f : Scalar ) : Scalar {
		return std.Math.acos(f);
	}

	public static inline function asin( f : Scalar ) : Scalar {
		return std.Math.asin(f);
	}

	public static inline function atan( f : Scalar ) : Scalar {
		return std.Math.atan(f);
	}

	public static inline function atan2( dx : Scalar, dy : Scalar ) : Scalar {
		return std.Math.atan2(dx,dy);
	}

	public static inline function min( a : Scalar, b : Scalar ) : Scalar {
		return a < b ? a : b;
	}

	public static inline function imin( a : Int, b : Int ) : Int {
		return a < b ? a : b;
	}

	public static inline function max( a : Scalar, b : Scalar ) : Scalar {
		return a > b ? a : b;
	}

	public static inline function imax( a : Int, b : Int ) : Int {
		return a > b ? a : b;
	}

	public static inline function floor( f : Scalar ) : Int {
		return std.Math.floor(f);
	}

	public static inline function abs( f : Scalar ) : Scalar {
		return f < 0 ? -f : f;
	}

	public static inline function sqrt( f : Scalar ) : Scalar {
		return std.Math.sqrt(f);
	}

	public static inline function invSqrt( f : Scalar ) : Scalar {
		return 1. / sqrt(f);
	}

	public static inline function clamp( v : Scalar, min : Scalar, max : Scalar ) {
		return Math.min(Math.max(v, min), max);
	}

	public static inline function degToRad( deg : Scalar ) {
		return deg * PI / 180.0;
	}

	public static inline function radToDeg( rad : Scalar ) {
		return rad * 180.0 / PI;
	}

	public static inline function isNaN( v : Scalar ) {
		return std.Math.isNaN(v);
	}

	public static inline function sign( v : Scalar ) {
 		return v >= 0.0 ? 1.0 : -1.0;
	}

	public static inline function square( v : Scalar ) {
		return v * v;
	}

	public static inline function localRayAABB( rayOrigin : Vec3, rayDirection : Vec3, min : Vec3, max : Vec3 ) : Scalar {
		var minT : Vec3 = (min - rayOrigin) / rayDirection;
		var maxT : Vec3 = (max - rayOrigin) / rayDirection;

		var realMin = Vec3.min(minT, maxT);
		var realMax = Vec3.max(minT, maxT);

		var minMax = Math.min( Math.min(realMax.x, realMax.y), realMax.z );
		var maxMin = Math.max( Math.max(realMin.x, realMin.y), realMin.z );

		if ( maxMin > minMax || minMax < 0.0 )
			return Math.SCALAR_MAX;

		return maxMin;
	}

	public static inline function localRayBox( rayOrigin : Vec3, rayDirection : Vec3, halfExtent : Vec3 ) : Scalar {
		return localRayAABB(rayOrigin, rayDirection, halfExtent.scaled(-1.0), halfExtent);
	}

	public static inline function localRaySphere( rayOrigin : Vec3, rayDirection : Vec3, radius : Scalar ) : Scalar {
		var m = rayOrigin;
		var c = m.dot(m) - radius * radius;

		if ( c < 0.0 )
			return Math.SCALAR_MAX;

		var b = m.dot(rayDirection);

		if ( b > 0.0 )
			return Math.SCALAR_MAX;

		var rayLenSq = rayDirection.lengthSq();

		var d = b * b - rayLenSq * c;

		if ( d < 0.0 || rayLenSq < Math.EPSILON )
			return Math.SCALAR_MAX;

		var t = -b - Math.sqrt(d);
		return t /= rayLenSq;
	}

	public static inline function localRayInfiniteCylinder( rayOrigin : Vec3, rayDirection : Vec3, radius : Scalar ) : Scalar {
		var fraction = Math.SCALAR_MAX;
		var originXY = new Vec3(rayOrigin.x, rayOrigin.y, 0.0);
		var originXYLenSq = originXY.lengthSq();
		var radius2 = radius * radius;
		if( originXYLenSq <= radius2 ) {
			fraction = 0.0;
		} else {
			var dirXY = new Vec3(rayDirection.x, rayDirection.y, 0.0);
			var a = dirXY.lengthSq();
			var b = 2.0 * originXY.dot(dirXY);
			var c = originXYLenSq - radius2;
			if ( a == 0.0 && b != 0.0 ) {
				fraction = -c / b;
			} else {
				var det = b * b - 4.0 * a * c;
				if ( det >= 0 ) {
					var q = (b + sign(b) * sqrt(det)) / -2.0;
					fraction = q / a;
					if ( q != 0.0 )
						fraction = Math.min(fraction, c / q);
				}
			}

			if ( fraction < 0.0 )
				fraction = Math.SCALAR_MAX;
		}

		return fraction;
	}

	public static inline function localRayCylinder( rayOrigin : Vec3, rayDirection : Vec3, halfHeight : Scalar, radius : Scalar ) : Scalar {
		var fraction = localRayInfiniteCylinder(rayOrigin, rayDirection, radius);
		if ( fraction == Math.SCALAR_MAX )
			return Math.SCALAR_MAX;

		if ( Math.abs(rayOrigin.z + fraction * rayDirection.z) <= halfHeight)
			return fraction;

		fraction = Math.SCALAR_MAX;
		var dirZ = rayDirection.z;
		if ( dirZ != 0.0 ) {
			var originZ = rayOrigin.z;
			var planeFraction = 0.0;
			if ( dirZ < 0.0 )
				planeFraction = (halfHeight - originZ) / dirZ;
			else
				planeFraction = -(halfHeight + originZ) / dirZ;
			if( planeFraction >= 0.0 ) {
				var point = rayOrigin + rayDirection * planeFraction;
				var distSq = point.x * point.x + point.y * point.y;
				if ( distSq <= radius * radius )
					fraction = planeFraction;
			}
		}

		return fraction;
	}

	public static inline function localRayCapsule( rayOrigin : Vec3, rayDirection : Vec3, halfHeight : Scalar, radius : Scalar ) : Scalar {
		var cylinder = localRayInfiniteCylinder(rayOrigin, rayDirection, radius);
		if ( cylinder == Math.SCALAR_MAX )
			return Math.SCALAR_MAX;

		if ( Math.abs(rayOrigin.z + cylinder * rayDirection.z) <= halfHeight )
			return cylinder;

		var sphereCenter = new Vec3(0.0, 0.0, halfHeight);
		var upper = localRaySphere(rayOrigin - sphereCenter, rayDirection, radius);
		var lower = localRaySphere(rayOrigin + sphereCenter, rayDirection, radius);

		return Math.min(upper, lower);
	}

	/**
		Möller–Trumbore intersection
	**/
	public static inline function rayTriangle( rayOrigin : Vec3, rayDirection : Vec3, v0 : Vec3, v1 : Vec3, v2 : Vec3 ) : Scalar {
		var e1 = v1 - v0;
		var e2 = v2 - v0;
		var rayCrossE2 = rayDirection.cross(e2);
		var det = e1.dot(rayCrossE2);
		if( det > -Math.EPSILON && det < Math.EPSILON )
			return Math.SCALAR_MAX;
		var invDet = 1.0 / det;
		var s = rayOrigin - v0;
		var u = s.dot(rayCrossE2) * invDet;
		var sCrossE1 = s.cross(e1);
		var v = rayDirection.dot(sCrossE1) * invDet;
		var t = e2.dot(sCrossE1) * invDet;
		if( u < 0.0 || v < 0.0 || u + v > 1.0 || t < Math.EPSILON )
			return Math.SCALAR_MAX;
		return t;
	}

	public static inline function computePointToLineDistance(linePointA : Vec3, linePointB : Vec3, point : Vec3) : Scalar {
		var distAB = (linePointB - linePointA).length();
		if (distAB < Math.EPSILON) {
			return (point - linePointA).length();
		}
		return ((point - linePointA).cross(point - linePointB)).length() / distAB;
	}

	public static inline function computePlaneSegmentIntersection( segA : Vec3, segB : Vec3, planeD : Scalar, planeNormal : Vec3 ) : Scalar {
		var parallelEpsilon : Scalar = 0.0001;
		var t : Scalar = -1.0;
		var nDotAB = planeNormal.dot(segB - segA);
		if (Math.abs(nDotAB) > parallelEpsilon)
			t = (planeD - planeNormal.dot(segA)) / nDotAB;
		return t;
	}

	public static inline function computeClosestPointOnSegment( segPointA : Vec3, segPointB : Vec3, pointC : Vec3 ) : Vec3 {
		var result = new Vec3();
		var ab = segPointB - segPointA;

		var abLengthSq = ab.lengthSq();
		if (abLengthSq < Math.EPSILON) {
			result.load(segPointA);
		} else {
			var t = (pointC - segPointA).dot(ab) / abLengthSq;
			if (t < 0.0)
				t = 0.0;
			if (t > 1.0)
				t = 1.0;

			result.load(segPointA + t * ab);
		}

		return result;
	}

	public static inline function computeClosestPointBetweenTwoSegments( seg1PointA : Vec3, seg1PointB : Vec3, seg2PointA : Vec3, seg2PointB : Vec3, closestPointSeg1 : Vec3, closestPointSeg2 : Vec3 ) {
		var d1 = seg1PointB - seg1PointA;
		var d2 = seg2PointB - seg2PointA;
		var r = seg1PointA - seg2PointA;
		var a = d1.lengthSq();
		var e = d2.lengthSq();
		var f = d2.dot(r);
		var s, t;

		if (a <= Math.EPSILON && e <= Math.EPSILON) {
			closestPointSeg1.load(seg1PointA);
			closestPointSeg2.load(seg2PointA);
			return;
		}
		if (a <= Math.EPSILON) {
			s = 0.0;
			t = Math.clamp(f / e, 0.0, 1.0);
		} else {
			var c = d1.dot(r);
			if (e <= Math.EPSILON) {
				t = 0.0;
				s = Math.clamp(-c / a, 0.0, 1.0);
			} else {
				var b = d1.dot(d2);
				var denom = a * e - b * b;
				s = denom == 0.0 ? 0.0 : Math.clamp((b * f - c * e) / denom, 0.0, 1.0);

				t = (b * s + f) / e;
				if (t < 0.0) {
					t = 0.0;
					s = Math.clamp(-c / a, 0.0, 1.0);
				}
				else if (t > 1.0) {
					t = 1.0;
					s = Math.clamp((b - c) / a, 0.0, 1.0);
				}
			}
		}
		closestPointSeg1.load(seg1PointA + d1 * s);
		closestPointSeg2.load(seg2PointA + d2 * t);
	}

	public static inline function getBaryCentricCoordinates3D(a : Vec3, b : Vec3, c : Vec3, out : Vec3) : Bool {
		var v0 = b - a;
		var v1 = c - a;
		var v2 = c - b;

		var d00 = v0.lengthSq();
		var d11 = v1.lengthSq();
		var d22 = v2.lengthSq();
		var onLine = true;
		if ( d00 <= d22 ) {
			var d01 = v0.dot(v1);

			var denom = d00 * d11 - d01 * d01;
			if( denom < 1.0e-12 ) {
				if ( d00 > d11 ) {
					getBaryCentricCoordinates2D(a, b, out);
					out.z = 0.0;
				} else {
					getBaryCentricCoordinates2D(a, c, out);
					out.z = out.y;
					out.y = 0.0;
				}
				onLine = false;
			} else {
				var a0 = a.dot(v0);
				var a1 = a.dot(v1);
				out.y = ( d01 * a1 - d11 * a0 ) / denom;
				out.z = ( d01 * a0 - d00 * a1 ) / denom;
				out.x = 1.0 - out.y - out.z;
			}
		} else {
			var d12 = v1.dot(v2);

			var denom = d11 * d22 - d12 * d12;
			if ( denom < 1.0e-10 ) {
				if ( d11 > d22 ) {
					getBaryCentricCoordinates2D(a, c, out);
					out.z = out.y;
					out.y = 0.0;
				} else {
					getBaryCentricCoordinates2D(b, c, out);
					out.z = out.y;
					out.y = out.x;
					out.x = 0.0;
				}
				onLine = false;
			} else {
				var c1 = c.dot(v1);
				var c2 = c.dot(v2);
				out.x = (d22 * c1 - d12 * c2) / denom;
				out.y = (d11 * c2 - d12 * c1) / denom;
				out.z = 1.0 - out.x - out.y;
			}
		}
		return onLine;
	}

	public static inline function getBaryCentricCoordinates2D(a : Vec3, b : Vec3, out : Vec3) : Bool {
		var ab = b - a;
		var denom = ab.lengthSq();
		var onLine = true;
		if ( denom < 1.0e-10 ) {
			if ( a.lengthSq() < b.lengthSq() )
				out.set(1.0, 0.0, 0.0);
			else
				out.set(0.0, 1.0, 0.0);
			onLine = false;
		} else {
			var v = -a.dot(ab) / denom;
			out.set(1.0 - v, v, 0.0);
		}
		return onLine;
	}
}
