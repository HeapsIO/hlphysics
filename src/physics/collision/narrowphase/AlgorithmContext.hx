package physics.collision.narrowphase;

typedef CollideFun = ( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) -> Bool;
typedef ShapecastFun = ( shapeCast : ShapeCast, shape2 : Shape, scale2 : Vec3, transform2 : Mat, collector : ShapeCastCollector ) -> Bool;
typedef RaycastFun = ( ray : Ray, shape : Shape, scale : Vec3, transform : Mat, tmpHit : HitResult, collector : RayCastCollector ) -> Bool;

class AlgorithmContext {

	var convexVsConvexAlgo : ConvexVsConvexAlgorithm;
	var convexVsMeshAlgo : ConvexVsMeshAlgorithm;
	var compoundAlgo : CompoundAlgorithm;
	var scaledAlgo : ScaledAlgorithm;
	var collisionMatrix : Array<Array<CollideFun>>;
	var shapecastMatrix : Array<Array<ShapecastFun>>;
	var raycastMatrix : Array<RaycastFun>;

	public function new() {
		convexVsConvexAlgo = new ConvexVsConvexAlgorithm();
		convexVsMeshAlgo = new ConvexVsMeshAlgorithm();
		compoundAlgo = new CompoundAlgorithm(this);
		scaledAlgo = new ScaledAlgorithm(this);
		collisionMatrix = [];
		shapecastMatrix = [];
		for( s1 in 0...ShapeCount ) {
			var t1 = (s1 : ShapeType);
			collisionMatrix[t1] = [];
			shapecastMatrix[t1] = [];
			for( s2 in 0...ShapeCount ) {
				var t2 = (s2 : ShapeType);
				var ftest = switch ([t1, t2]) {
					case [Sphere, Sphere]: SphereVsSphereAlgorithm.testCollision;
					case [Sphere, Capsule]: SphereVsCapsuleAlgorithm.testCollision;
					case [Capsule, Sphere]: SphereVsCapsuleAlgorithm.testCollision2;
					case [Capsule, Capsule]: CapsuleVsCapsuleAlgorithm.testCollision;
					case [Mesh, _]: null;
					case [_, Mesh] if( t1.isConvex() ): convexVsMeshAlgo.testCollision;
					case [Compound, _]: compoundAlgo.collideCompoundVsShape;
					case [_, Compound]: compoundAlgo.collideShapeVsCompound;
					case [Scaled, _]: scaledAlgo.collideScaledVsShape;
					case [_, Scaled]: scaledAlgo.collideShapeVsScaled;
					case [Empty, _]: EmptyAlgorithm.collideEmptyVsShape;
					case [_, Empty]: EmptyAlgorithm.collideShapeVsEmpty;
					default:
						if( t1.isConvex() && t2.isConvex() )
							convexVsConvexAlgo.testCollision;
						else
							throw "Don't know how to handle collision pair " + t1 + " + " + t2;
				}
				collisionMatrix[t1][t2] = ftest;
				var fcast : ShapecastFun = switch ([t1, t2]) {
					case [Mesh, Mesh]: null;
					case [Mesh, _]: null; // TODO
					case [_, Mesh] if( t1.isConvex() ): convexVsMeshAlgo.shapecast;
					case [Compound, _]: compoundAlgo.shapecastCompoundVsShape;
					case [_, Compound]: compoundAlgo.shapecastShapeVsCompound;
					case [Scaled, _]: scaledAlgo.shapecastScaledVsShape;
					case [_, Scaled]: scaledAlgo.shapecastShapeVsScaled;
					case [Empty, _]: EmptyAlgorithm.shapecastEmptyVsShape;
					case [_, Empty]: EmptyAlgorithm.shapecastShapeVsEmpty;
					default:
						if( t1.isConvex() && t2.isConvex() )
							convexVsConvexAlgo.shapecast;
						else
							throw "Don't know how to handle shapecast pair " + t1 + " + " + t2;
				}
				shapecastMatrix[t1][t2] = fcast;
			}
		}
		raycastMatrix = [];
		for( s1 in 0...ShapeCount ) {
			var t1 = (s1 : ShapeType);
			var fray : RaycastFun = switch (t1) {
				case Mesh: convexVsMeshAlgo.raycast;
				case Compound: compoundAlgo.raycast;
				case Scaled: scaledAlgo.raycast;
				default: null;
			}
			raycastMatrix[t1] = fray;
		}
	}

	public inline function collide( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) : Bool {
		var t1 = shape1.getType();
		var t2 = shape2.getType();
		var fun : CollideFun = collisionMatrix[t1][t2];
		if( fun == null )
			throw "Can't collide between " + t1 + " + " + t2;
		return fun(shape1, shape2, scale1, scale2, transform1, transform2, collector);
	}

	public inline function shapecast( shapeCast : ShapeCast, shape2 : Shape, scale2 : Vec3, transform2 : Mat, collector : ShapeCastCollector ) : Bool {
		if( shapeCast.shape == shape2 )
			return false;
		var t1 = shapeCast.shape.getType();
		var t2 = shape2.getType();
		var fun : ShapecastFun = shapecastMatrix[t1][t2];
		if( fun == null )
			throw "Can't shapecast between " + t1 + " + " + t2;
		return fun(shapeCast, shape2, scale2, transform2, collector);
	}

	public inline function raycast( ray : Ray, shape : Shape, scale : Vec3, transform : Mat, tmpHit : HitResult, collector : RayCastCollector ) : Bool {
		var t1 = shape.getType();
		var fun : RaycastFun = raycastMatrix[t1];
		if( fun == null ) {
			var b = shape.raycast(ray, scale, transform, tmpHit);
			if( b )
				collector.addHit(tmpHit.position, tmpHit.normal, tmpHit.fraction);
			return b;
		}
		return fun(ray, shape, scale, transform, tmpHit, collector);
	}
}
