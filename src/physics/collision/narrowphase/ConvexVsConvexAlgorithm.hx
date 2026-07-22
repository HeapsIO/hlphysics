package physics.collision.narrowphase;

class TransformedConvexSupport extends ConvexSupport {
	public var support : ConvexSupport;
	@:packed public var transform : Mat;
	@:packed var tmpdir : Vec3;

	public function init( shape : Shape, scale : Vec3 ) {
		throw "Should probably not be called and not implemented";
	}

	public inline function initWithCache( support : ConvexSupport, transform : Mat ) {
		this.support = support;
		this.transform.load(transform);
	}

	public inline function getSupportWithMargin( dir : Vec3, out : Vec3 ) : Void {
		tmpdir.load(dir.transformed3x3Transposed(transform));
		support.getSupportWithMargin(tmpdir, out);
		out.transform(transform);
	}

	public inline function getSupport( dir : Vec3, out : Vec3 ) : Void {
		tmpdir.load(dir.transformed3x3Transposed(transform));
		support.getSupport(tmpdir, out);
		out.transform(transform);
	}

	public inline function getMargin() : Scalar {
		return support.getMargin();
	}
}

class ConvexVsConvexAlgorithm {
	static inline final MAX_POINT_TO_INCLUDE_ORIGIN_IN_HULL = 32;

	var gjk : GJK;
	var hull : EPAConvexHullBuilder;

	var supportCache1 : Map<ShapeType, ConvexSupport>;
	var supportCache2 : Map<ShapeType, ConvexSupport>;

	@:packed var body2ToBody1 : Mat;
	@:packed var penetrationAxis : Vec3;
	@:packed var a : Vec3;
	@:packed var b : Vec3;
	@:packed var shapeCastDirection : Vec3;

	@:packed var tmpVec : Vec3;

	var transformedSupport : TransformedConvexSupport;

	public function new() {
		gjk = new GJK();
		hull = new EPAConvexHullBuilder();
		supportCache1 = new Map();
		supportCache2 = new Map();
		transformedSupport = new TransformedConvexSupport();
	}

	static function getSupportWithCache( supportCache : Map<ShapeType, ConvexSupport>, shape : ConvexShape, scale : Vec3 ) : ConvexSupport {
		var key : ShapeType = shape.getType();
		var support = supportCache.get(key);
		if( support == null ) {
			support = Type.createEmptyInstance(shape.getSupportClass());
			supportCache.set(key, support);
		}
		support.init(shape, scale);
		return support;
	}

	inline function getSupportWithCache1( shape : ConvexShape, scale : Vec3 ) : ConvexSupport {
		return getSupportWithCache(supportCache1, shape, scale);
	}

	inline function getSupportWithCache2( shape : ConvexShape, scale : Vec3 ) : ConvexSupport {
		return getSupportWithCache(supportCache2, shape, scale);
	}

	public function testCollision( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) {
		var shape1 : ConvexShape = hl.Api.unsafeCast(shape1);
		var shape2 : ConvexShape = hl.Api.unsafeCast(shape2);
		body2ToBody1.multiply3x4inline(transform2, transform1.getInverse());

		penetrationAxis.load(body2ToBody1.getPosition());
		if ( penetrationAxis.lengthSq() < Math.EPSILON )
			penetrationAxis.set(1.0, 0.0, 0.0);

		a.set(0.0, 0.0, 0.0);
		b.set(0.0, 0.0, 0.0);
		var support1 = getSupportWithCache1(shape1, scale1);
		var support2 = getSupportWithCache2(shape2, scale2);
		transformedSupport.initWithCache(support2, body2ToBody1);
		if ( getPenetrationDepth(support1, transformedSupport, penetrationAxis, a, b) ) {
			a.transform(transform1);
			b.transform(transform1);
			penetrationAxis.transform3x3(transform1);

			collector.addHit(a, b, penetrationAxis.normalized(), (b - a).length());
			return true;
		}
		return false;
	}

	public function shapecast( shapeCast : ShapeCast, shape2 : Shape, scale2 : Vec3, transform2 : Mat, collector : ShapeCastCollector ) : Bool {
		var shape1 : ConvexShape = hl.Api.unsafeCast(shapeCast.shape);
		var shape2 : ConvexShape = hl.Api.unsafeCast(shape2);
		var invTransform2 = transform2.getInverse();
		var transform1 = shapeCast.transform * invTransform2;

		shapeCastDirection.load(shapeCast.direction);
		shapeCastDirection.transform3x3(invTransform2);

		var support1 = getSupportWithCache1(shape1, shapeCast.scale);
		var support2 = getSupportWithCache2(shape2, scale2);
		transformedSupport.initWithCache(support1, transform1);
		var maxFraction = collector.getCurrentMaxFraction();
		var lambda = gjk.shapecast(shapeCastDirection, transformedSupport, support2, maxFraction, shapeCast.collisionTolerance, penetrationAxis, a, b);
		if ( lambda == Math.SCALAR_MAX )
			return false;

		var contactNormalInvalid = penetrationAxis.isNearZero();
		if ( lambda == 0.0 && (contactNormalInvalid || transformedSupport.getMargin() + support2.getMargin() == 0.0) ) {
			if ( !getPenetrationDepthEPA(transformedSupport, support2, penetrationAxis, a, b) )
				penetrationAxis.load(shapeCastDirection);
		} else if ( contactNormalInvalid ) {
			penetrationAxis.load(shapeCastDirection);
		}

		a.transform(transform2);
		b.transform(transform2);
		penetrationAxis.transform3x3(transform2);
		collector.addHit(a, b, penetrationAxis, (b - a).length(), lambda);
		return true;
	}

	function getPenetrationDepth( shape1 : ConvexSupport, shape2 : ConvexSupport, v : Vec3, a : Vec3, b : Vec3 ) : Bool {
		var result = gjk.getPenetrationDepthGJK(shape1, shape2, v, a, b);
		switch(result) {
		case CollideInMargin:
			return true;
		case Seperated:
			return false;
		case Interpenetrate:
			return getPenetrationDepthEPA(shape1, shape2, v, a, b);
		}
	}

	function getPenetrationDepthEPA( shape1 : ConvexSupport, shape2 : ConvexSupport, v : Vec3, a : Vec3, b : Vec3 ) : Bool {
		var tolerance = 1.0e-4;
		var pointCount = gjk.pointCount;
		var P = gjk.P;
		var Q = gjk.Q;
		var Y = gjk.Y;

		inline function addSupportPoint(dir : Vec3) : Vec3 {
			tmpVec.load(dir);
			var i = pointCount++;
			var p = P[i];
			var q = Q[i];
			var w = Y[i];
			shape1.getSupportWithMargin(tmpVec, p);
			tmpVec.scale(-1);
			shape2.getSupportWithMargin(tmpVec, q);
			w.load(p - q);
			return w;
		}

		switch( pointCount ) {
		case 1:
			pointCount--;
			addSupportPoint(new Vec3(0, 1, 0));
			addSupportPoint(new Vec3(-1, -1, -1));
			addSupportPoint(new Vec3(1, -1, -1));
			addSupportPoint(new Vec3(0, 1, 1));
		case 2:
			var axis = (Y[1].subbed(Y[0])).normalized();
			var rot = new Mat();
			rot.initRotationAxis(axis, Math.degToRad(120.0));
			var dir1 = axis.getNormalizedPerpendicular();
			var dir2 = dir1.transformed3x3(rot);
			var dir3 = dir2.transformed3x3(rot);
			addSupportPoint(dir1);
			addSupportPoint(dir2);
			addSupportPoint(dir3);
		default:
		}

		hull.reset(Y);
		hull.initialize(0, 1, 2);
		for ( i in 3...pointCount ) {
			var distSq : Scalar = 0.0;
			var t = hull.findFacingTriangle(Y[i], { bestDistSq : distSq });
			if( t != null ) {
				if ( !hull.addPoint(t, i, Math.SCALAR_MAX))
					return false;
			}
		}

		while(true) {
			var t = hull.peekClosestTriangleInQueue();
			if ( t.removed ) {
				hull.popClosestTriangleFromQueue();

				if (!hull.hasNextTriangle())
					return false;

				hull.freeTriangle(t);
				continue;
			}

			if ( t.closestLenSq >= 0.0 )
				break;

			hull.popClosestTriangleFromQueue();

			var newIdx : Int = pointCount;
			var w = addSupportPoint(t.normal).clone();

			if ( !t.isFacing(w) || !hull.addPoint(t, newIdx, Math.SCALAR_MAX))
				return false;

			hull.freeTriangle(t);

			if (!hull.hasNextTriangle() || pointCount >= MAX_POINT_TO_INCLUDE_ORIGIN_IN_HULL )
				return false;
		}

		var closestDistSq = Math.SCALAR_MAX;
		var last : EPAConvexHullBuilder.Triangle = null;
		var flipVSign = false;

		do {
			var t = hull.popClosestTriangleFromQueue();
			if ( t.removed ) {
				hull.freeTriangle(t);
				continue;
			}

			if ( t.closestLenSq >= closestDistSq )
				break;

			if ( last != null )
				hull.freeTriangle(last);
			last = t;

			var newIdx = pointCount;
			var w = addSupportPoint(t.normal).clone();
			var dot = t.normal.dot(w);
			if (dot < 0.0)
				return false;

			var distSq = Math.square(dot) / t.normal.lengthSq();
			if ( distSq - t.closestLenSq < t.closestLenSq * tolerance )
				break;

			if ( !t.isFacing(w) )
				break;

			closestDistSq = Math.min(closestDistSq, distSq);
			if ( !hull.addPoint(t, newIdx, closestDistSq))
				break;

			var hasDefect = false;
			for ( nt in hull.newTriangles ) {
				if ( nt.isFacingOrigin() ) {
					hasDefect = true;
					break;
				}
			}
			if ( hasDefect ) {
				var sNormal = t.normal.scaled(-1.0);
				tmpVec.load(sNormal);
				var w2 = new Vec3();
				shape1.getSupportWithMargin(tmpVec, tmpVec);
				w2.load(tmpVec);
				tmpVec.load(t.normal);
				shape2.getSupportWithMargin(tmpVec, tmpVec);
				w2.sub(tmpVec);
				var dot2 = sNormal.dot(w2);
				if ( dot2 < dot)
					flipVSign = true;
				break;
			}
		} while( hull.hasNextTriangle() && pointCount < GJK.MAX_POINT);

		if ( last == null )
			return false;

		v.load(last.centroid.dot(last.normal) * (1.0 / last.normal.lengthSq()) * last.normal);

		if ( v.isNearZero() )
			return false;

		if ( flipVSign )
			v.scale(-1);

		var p0 = P[last.edge0.startIdx];
		var p1 = P[last.edge1.startIdx];
		var p2 = P[last.edge2.startIdx];

		var q0 = Q[last.edge0.startIdx];
		var q1 = Q[last.edge1.startIdx];
		var q2 = Q[last.edge2.startIdx];

		if ( last.lambdaRelativeTo0 ) {
			a.load(p0 + last.lambdaX * (p1 - p0) + last.lambdaY * (p2 - p0));
			b.load(q0 + last.lambdaX * (q1 - q0) + last.lambdaY * (q2 - q0));
		} else {
			a.load(p1 + last.lambdaX * (p0 - p1) + last.lambdaY * (p2 - p1));
			b.load(q1 + last.lambdaX * (q0 - q1) + last.lambdaY * (q2 - q1));
		}

		return true;
	}
}
