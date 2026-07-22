package physics.collision.narrowphase;

enum GJKResult {
	Seperated;
	CollideInMargin;
	Interpenetrate;
}

class GJK {
	public static inline final MAX_POINT = 128;

	public var Y : hl.CArray<Vec3>;
	public var Q : hl.CArray<Vec3>;
	public var P : hl.CArray<Vec3>;
	public var pointCount : Int;

	var p : Vec3 = new Vec3();
	var q : Vec3 = new Vec3();
	var w : Vec3 = new Vec3();

	var tmpVec = new Vec3();

	public function new() {
		Y = hl.CArray.alloc(Vec3Impl, MAX_POINT);
		Q = hl.CArray.alloc(Vec3Impl, MAX_POINT);
		P = hl.CArray.alloc(Vec3Impl, MAX_POINT);
	}

	public function getPenetrationDepthGJK( shape1 : ConvexSupport, shape2 : ConvexSupport, v : Vec3, a : Vec3, b : Vec3 ) : GJKResult {
		pointCount = 0;

		var margin = shape1.getMargin() + shape2.getMargin();
		var marginSq = margin * margin;

		var vLenSq : Scalar = v.lengthSq();
		var prevVLenSq = Math.SCALAR_MAX;
		var noIntersection = false;
		var simplexIsFull = false;
		var simplexMaxLength = false;

		p.set(0.0, 0.0, 0.0);
		q.set(0.0, 0.0, 0.0);
		w.set(0.0, 0.0, 0.0);
		while( true ) {
			shape1.getSupport(v, p);
			tmpVec.load(v.scaled(-1.0));
			shape2.getSupport(tmpVec, q);
			w.load(p - q);

			var vDotW = v.dot(w);

			if ( vDotW < 0.0 && vDotW * vDotW > vLenSq * marginSq ) {
				vLenSq = Math.SCALAR_MAX;
				break;
			}

			P[pointCount].load(p);
			Q[pointCount].load(q);
			Y[pointCount].load(w);
			pointCount++;

			var set = 0;
			var out = getClosest(prevVLenSq, Y, pointCount, v);
			if( out.hasResult ) {
				set = out.set;
				vLenSq = out.vLenSq;
			}
			if ( !out.hasResult ) {
				pointCount--;
				break;
			}

			if( set == 0xf ) {
				v.set(0.0, 0.0, 0.0);
				vLenSq = 0.0;
				break;
			}

			updatePointSetYPQ(set);

			var toleranceSq = 9.99999905e-09;
			if ( vLenSq <= toleranceSq ) {
				v.set(0.0, 0.0, 0.0);
				vLenSq = 0.0;
			}

			var maxYLengthSq = Y[0].lengthSq();
			for ( i in 1...pointCount )
				maxYLengthSq = Math.max(Y[i].lengthSq(), maxYLengthSq);

			if( vLenSq <= Math.EPSILON * maxYLengthSq) {
				v.set(0.0, 0.0, 0.0);
				vLenSq = 0.0;
				break;
			}

			v.scale(-1.0);

			if ( prevVLenSq - vLenSq <= Math.EPSILON * prevVLenSq )
				break;
			prevVLenSq = vLenSq;
		};

		if ( vLenSq > marginSq )
			return Seperated;

		calculatePointAAndB(a, b);

		if ( vLenSq > 0.0 ) {
			var vLen = Math.sqrt(vLenSq);
			a.add(v * (shape1.getMargin() / vLen));
			b.sub(v * (shape2.getMargin() / vLen));
			return CollideInMargin;
		}

		return Interpenetrate;
	}

	public function shapecast( direction : Vec3, shape1 : ConvexSupport, shape2 : ConvexSupport, maxFraction : Scalar, tolerance : Scalar, v : Vec3, a : Vec3, b : Vec3 ) : Scalar {
		var toleranceSq = tolerance * tolerance;

		var convexRadius1 = shape1.getMargin();
		var convexRadius2 = shape2.getMargin();

		var sumConvexRadius = convexRadius1 + convexRadius2;
		pointCount = 0;

		var lambda = 0.0;

		var x = new Vec3();
		tmpVec.set(0.0, 0.0, 0.0);
		shape1.getSupport(tmpVec, p);
		shape2.getSupport(tmpVec, q);
		v.load(p - q);

		var vLenSq = Math.SCALAR_MAX;
		var allowRestart = false;

		var prevV = new Vec3();

		p.set(0.0, 0.0, 0.0);
		q.set(0.0, 0.0, 0.0);
		w.set(0.0, 0.0, 0.0);
		while( true ) {
			tmpVec.load(v.scaled(-1));
			shape1.getSupport(tmpVec, p);
			tmpVec.load(v);
			shape2.getSupport(tmpVec, q);
			w.load(x - (q - p));

			var vDotW = v.dot(w) - sumConvexRadius * v.length();

			if( vDotW > 0.0 ) {
				var vDotR = v.dot(direction);

				if( vDotR >= -1.0e-18)
					return Math.SCALAR_MAX;

				var delta = vDotW / vDotR;
				var oldLambda = lambda;
				lambda -= delta;

				if (oldLambda == lambda )
					break;

				if (lambda >= maxFraction)
					return Math.SCALAR_MAX;

				x.load(lambda * direction);

				vLenSq = Math.SCALAR_MAX;

				toleranceSq = Math.square(tolerance + sumConvexRadius);

				allowRestart = true;
			}

			P[pointCount].load(p);
			Q[pointCount].load(q);
			pointCount++;
			for ( i in 0...pointCount )
				Y[i].load(x - (Q[i] - P[i]));

			var set = 0;
			var out = getClosest(vLenSq, Y, pointCount, v);
			if( out.hasResult ) {
				set = out.set;
				vLenSq = out.vLenSq;
			}
			if ( !out.hasResult ) {
				if (!allowRestart)
					break;

				allowRestart = false;
				P[0].load(p);
				Q[0].load(q);
				pointCount = 1;
				v.load(x - q);
				vLenSq = Math.SCALAR_MAX;
				continue;
			} else if (set == 0xf) {
				break;
			}

			updatePointSetPQ(set);

			if (vLenSq <= toleranceSq)
				break;

			prevV.load(v);
		}

		for ( i in 0...pointCount )
			Y[i].load(x - (Q[i] - P[i]));

		var normalizedV = v.normalizedOr(Vec3.zero());
		var convexRadiusA = convexRadius1 * normalizedV;
		var convexRadiusB = convexRadius2 * normalizedV;

		switch( pointCount) {
		case 1:
			b.load(Q[0] + convexRadiusB);
			if( lambda > 0.0 )
				a.load(b);
			else
				a.load(P[0] - convexRadiusA);
		case 2:
			var coord = new Vec3();
			Math.getBaryCentricCoordinates2D(Y[0], Y[1], coord);
			b.load(coord.x * Q[0] + coord.y * Q[1] + convexRadiusB);
			if( lambda > 0.0 )
				a.load(b);
			else
				a.load(coord.x * P[0] + coord.y * P[1] - convexRadiusA);
		case 3,4:
			var coord = new Vec3();
			Math.getBaryCentricCoordinates3D(Y[0], Y[1], Y[2], coord);
			b.load(coord.x * Q[0] + coord.y * Q[1] + coord.z * Q[2] + convexRadiusB);
			if( lambda > 0.0 )
				a.load(b);
			else
				a.load(coord.x * P[0] + coord.y * P[1] + coord.z * P[2] - convexRadiusA);
		}

		if ( sumConvexRadius == 0.0 )
			v.load(prevV);
		v.scale(-1.0);

		return lambda;
	}

	static inline function originOutsideOfTetrahedronPlanes( a : Vec3, b : Vec3, c : Vec3, d : Vec3 ) : Vec4 {
		var ab = b - a;
		var ac = c - a;
		var ad = d - a;
		var bd = d - b;
		var bc = c - b;

		var abCrossAc = ab.cross(ac);
		var acCrossAd = ac.cross(ad);
		var adCrossAb = ad.cross(ab);
		var bdCrossBc = bd.cross(bc);

		var signp0 = a.dot(abCrossAc);
		var signp1 = a.dot(acCrossAd);
		var signp2 = a.dot(adCrossAb);
		var signp3 = b.dot(bdCrossBc);

		var signd0 = ad.dot(abCrossAc);
		var signd1 = ab.dot(acCrossAd);
		var signd2 = ac.dot(adCrossAb);
		var signd3 = -ab.dot(bdCrossBc);

		var signBits = ( signd0 < 0 ? 1 : 0 ) | ( signd1 < 0 ? 2 : 0 ) | ( signd2 < 0 ? 4 : 0 ) | ( signd3 < 0 ? 8 : 0 );
		var result = new Vec4();
		switch(signBits) {
		case 0:
			result.set(signp0 >= -Math.EPSILON ? 1.0 : -1.0, signp1 >= -Math.EPSILON ? 1.0 : -1.0, signp2 >= -Math.EPSILON ? 1.0 : -1.0, signp3 >= -Math.EPSILON ? 1.0 : -1.0);
		case 0xf:
			result.set(signp0 <= Math.EPSILON ? 1.0 : -1.0, signp1 <= Math.EPSILON ? 1.0 : -1.0, signp2 <= Math.EPSILON ? 1.0 : -1.0, signp3 <= Math.EPSILON ? 1.0 : -1.0);
		default:
			result.set(1.0, 1.0, 1.0, 1.0);
		}
		return result;
	}

	static inline function getClosestPointOnTetrahedron( a : Vec3, b : Vec3, c : Vec3, d : Vec3, out : Vec3 ) : Int {
		var closestSet = 0b1111;
		var closestPoint = new Vec3();
		var bestDistSq = Math.SCALAR_MAX;

		var originOutOfPlanes = originOutsideOfTetrahedronPlanes(a, b, c, d);

		if ( originOutOfPlanes.x > 0.0) {
			closestSet = 0b0001;
			closestPoint.load(a);
			bestDistSq = closestPoint.lengthSq();
		}

		if ( originOutOfPlanes.y > 0.0 ) {
			var q = new Vec3();
			var set = getClosestPointOnTriangle(a, c, d, q);
			var distSq = q.lengthSq();
			if( distSq < bestDistSq ) {
				bestDistSq = distSq;
				closestPoint.load(q);
				closestSet = ( set & 0b0001 ) + ((set & 0b0110) << 1);
			}
		}

		if ( originOutOfPlanes.z > 0.0 ) {
			var q = new Vec3();
			var set = getClosestPointOnTriangle(a, b, d, q);
			var distSq = q.lengthSq();
			if ( distSq < bestDistSq ) {
				bestDistSq = distSq;
				closestPoint.load(q);
				closestSet = (set & 0b0011) + ((set & 0b0100) << 1);
			}
		}

		if( originOutOfPlanes.w > 0.0 ) {
			var q = new Vec3();
			var set = getClosestPointOnTriangle(b, c, d, q);
			var distSq = q.lengthSq();
			if( distSq < bestDistSq ) {
				closestPoint.load(q);
				closestSet = set << 1;
			}
		}

		out.load(closestPoint);
		return closestSet;
	}

	static inline function getClosestPointOnTriangle( inA : Vec3, inB : Vec3, inC : Vec3, out : Vec3 ) : Int {
		var bcDot = 0.0;
		var acDot = 0.0;
		{
			var ac = inC - inA;
			var bc = inC - inB;
			bcDot = bc.dot(bc);
			acDot = ac.dot(ac);
		}
		var a = new Vec3();
		var b = inB;
		var c = new Vec3();
		var swapAC = bcDot < acDot;
		if ( swapAC ) {
			a.load(inC);
			c.load(inA);
		} else {
			a.load(inA);
			c.load(inC);
		}

		var ab = b - a;
		var ac = c - a;
		var n = ab.cross(ac);
		var nLenSq = n.lengthSq();

		if( nLenSq < 1.0e-10 ) {

			var closestSet = 0b0100;
			var closestPoint = inC.clone();
			var bestDistSq = inC.lengthSq();

			var aLenSq = inA.lengthSq();
			if ( aLenSq < bestDistSq ) {
				closestSet = 0b0001;
				closestPoint.load(inA);
				bestDistSq = aLenSq;
			}

			var bLenSq = b.lengthSq();
			if ( bLenSq < bestDistSq ) {
				closestSet = 0b0010;
				closestPoint.load(b);
				bestDistSq = bLenSq;
			}

			var acLenSq = ac.lengthSq();
			if ( acLenSq > Math.EPSILON * Math.EPSILON ) {
				var v = Math.clamp(-a.dot(ac) / acLenSq, 0.0, 1.0);
				var q = a + v * ac;
				var distSq = q.lengthSq();
				if ( distSq < bestDistSq ) {
					closestSet = 0b0101;
					closestPoint.load(q);
					bestDistSq = distSq;
				}
			}

			var bc = inC - b;
			var bcLenSq = bc.lengthSq();
			if ( bcLenSq > Math.EPSILON * Math.EPSILON ) {
				var v = Math.clamp(-b.dot(bc) / bcLenSq, 0.0, 1.0);
				var q = b + v * bc;
				var distSq = q.lengthSq();
				if ( distSq < bestDistSq ) {
					closestSet = 0b0110;
					closestPoint.load(q);
					bestDistSq = distSq;
				}
			}

			ab.load(b - inA);
			var abLenSq = ab.lengthSq();
			if( abLenSq > Math.EPSILON * Math.EPSILON ) {
				var v = Math.clamp(-inA.dot(ab) / abLenSq, 0.0, 1.0);
				var q = inA + v * ab;
				var distSq = q.lengthSq();
				if ( distSq < bestDistSq ) {
					closestSet = 0b0011;
					closestPoint.load(q);
					bestDistSq = distSq;
				}
			}

			out.load(closestPoint);
			return closestSet;
		}

		var ap = a.scaled(-1.0);
		var d1 = ab.dot(ap);
		var d2 = ac.dot(ap);
		if( d1 <= 0.0 && d2 <= 0.0 ) {
			out.load(a);
			return swapAC ? 0b0100 : 0b0001;
		}

		var bp = b.scaled(-1.0);
		var d3 = ab.dot(bp);
		var d4 = ac.dot(bp);
		if( d3 >= 0.0 && d4 <= d3) {
			out.load(b);
			return 0b0010;
		}

		if ( d1 * d4 <= d3 * d2 && d1 >= 0.0 && d3 <= 0.0) {
			var v = d1 / (d1 - d3);
			out.load(a + v * ab);
			return swapAC ? 0b0110 : 0b0011;
		}

		var cp = c.scaled(-1);
		var d5 = ab.dot(cp);
		var d6 = ac.dot(cp);
		if ( d6 >= 0.0 && d5 <= d6 ) {
			out.load(c);
			return swapAC ? 0b0001 : 0b0100;
		}

		if ( d5 * d2 <= d1 * d6 && d2 >= 0.0 && d6 <= 0.0 ) {
			var w = d2 / (d2 - d6);
			out.load(a + w * ac);
			return 0b0101;
		}

		var d4_d3 = d4 - d3;
		var d5_d6 = d5 - d6;
		if ( d3 * d6 <= d5 * d4 && d4_d3 >= 0.0 && d5_d6 >= 0.0 ) {
			var w = d4_d3 / (d4_d3 + d5_d6);
			out.load(b + w * (c - b));
			return swapAC ? 0b0011 : 0b0110;
		}

		out.load( n * (a + b + c).dot(n) * (1.0 / (3.0 * nLenSq)) );
		return 0b0111;
	}

	static inline function getClosestPointOnLine( a : Vec3, b : Vec3, out : Vec3 ) : Int {
		var uv = new Vec3();
		Math.getBaryCentricCoordinates2D(a, b, uv);
		if ( uv.y <= 0.0 ) {
			out.load(a);
			return 0b0001;
		}
		if ( uv.x <= 0.0 ) {
			out.load(b);
			return 0b0010;
		}
		out.load(a * uv.x + b * uv.y);
		return 0b0011;
	}

	static inline function getClosest( prevVLenSq : Scalar, Y : hl.CArray<Vec3>, pointCount : Int, outV : Vec3 ) : { hasResult : Bool, vLenSq : Scalar, set : Int } {
		var set;
		var v = new Vec3();

		switch ( pointCount ) {
		case 1:
			set = 0b0001;
			v.load(Y[0]);
		case 2:
			set = getClosestPointOnLine(Y[0], Y[1], v);
		case 3:
			set = getClosestPointOnTriangle(Y[0], Y[1], Y[2], v);
		case 4:
			set = getClosestPointOnTetrahedron(Y[0], Y[1], Y[2], Y[3], v);
		default:
			throw "assert";
		}

		var vLenSq = v.lengthSq();
		var hasResult = false;
		if ( vLenSq < prevVLenSq ) {
			outV.load(v);
			hasResult = true;
		}
		return { hasResult : hasResult, vLenSq : vLenSq, set : set };
	}

	function updatePointSetPQ( set : Int ) {
		var newPointCount = 0;
		for ( i in 0...pointCount ) {
			if ( (set & (1 << i)) != 0 ) {
				P[newPointCount].load(P[i]);
				Q[newPointCount].load(Q[i]);
				newPointCount++;
			}
		}
		pointCount = newPointCount;
	}

	function updatePointSetYPQ( set : Int ) {
		var newPointCount = 0;
		for ( i in 0...pointCount ) {
			if ( (set & (1 << i)) != 0 ) {
				Y[newPointCount].load(Y[i]);
				P[newPointCount].load(P[i]);
				Q[newPointCount].load(Q[i]);
				newPointCount++;
			}
		}
		pointCount = newPointCount;
	}

	inline function calculatePointAAndB( a : Vec3, b : Vec3 ) {
		switch (pointCount) {
		case 1:
			a.load(P[0]);
			b.load(Q[0]);
		case 2:
			var coord = new Vec3();
			Math.getBaryCentricCoordinates2D(Y[0], Y[1], coord);
			a.load(coord.x * P[0] + coord.y * P[1]);
			b.load(coord.x * Q[0] + coord.y * Q[1]);
		case 3:
			var coord = new Vec3();
			Math.getBaryCentricCoordinates3D(Y[0], Y[1], Y[2], coord);
			a.load(coord.x * P[0] + coord.y * P[1] + coord.z * P[2]);
			b.load(coord.x * Q[0] + coord.y * Q[1] + coord.z * Q[2]);
		default:
		}
	}
}
