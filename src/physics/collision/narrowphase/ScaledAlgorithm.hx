package physics.collision.narrowphase;

class ScaledAlgorithm {

	var algoCtx : AlgorithmContext;
	var freeVec : Pool<Vec3Impl>;
	var freeShapecast : Pool<ShapeCast>;

	public function new( algoCtx : AlgorithmContext ) {
		this.algoCtx = algoCtx;
		this.freeVec = new Pool(Vec3Impl, () -> Vec3.zero());
		this.freeShapecast = new Pool(ShapeCast, () -> new ShapeCast());
	}

	public function collideShapeVsScaled( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) : Bool {
		var shape2 : ScaledShape = hl.Api.unsafeCast(shape2);
		var tmpScale2 = freeVec.get();
		tmpScale2.load(scale2.multiplied(shape2.scale));
		var b = algoCtx.collide(shape1, shape2.shape, scale1, tmpScale2, transform1, transform2, collector);
		freeVec.put(tmpScale2);
		return b;
	}

	public function collideScaledVsShape( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) : Bool {
		var shape1 : ScaledShape = hl.Api.unsafeCast(shape1);
		var tmpScale1 = freeVec.get();
		tmpScale1.load(scale1.multiplied(shape1.scale));
		var b = algoCtx.collide(shape1.shape, shape2, tmpScale1, scale2, transform1, transform2, collector);
		freeVec.put(tmpScale1);
		return b;
	}

	public function shapecastShapeVsScaled( shapeCast : ShapeCast, shape2 : Shape, scale2 : Vec3, transform2 : Mat, collector : ShapeCastCollector ) : Bool {
		var shape2 : ScaledShape = hl.Api.unsafeCast(shape2);
		var tmpScale2 = freeVec.get();
		tmpScale2.load(scale2.multiplied(shape2.scale));
		var b = algoCtx.shapecast(shapeCast, shape2.shape, tmpScale2, transform2, collector);
		freeVec.put(tmpScale2);
		return b;
	}

	public function shapecastScaledVsShape( shapeCast : ShapeCast, shape2 : Shape, scale2 : Vec3, transform2 : Mat, collector : ShapeCastCollector ) : Bool {
		var shape1 : ScaledShape = hl.Api.unsafeCast(shapeCast.shape);
		var subShapeCast = freeShapecast.get();
		subShapeCast.shape = shape1.shape;
		subShapeCast.direction = shapeCast.direction;
		subShapeCast.transform = shapeCast.transform;
		subShapeCast.scale = shapeCast.scale.multiplied(shape1.scale);
		var b = algoCtx.shapecast(subShapeCast, shape2, scale2, transform2, collector);
		freeShapecast.put(subShapeCast);
		return b;
	}

	public function raycast( ray : Ray, shape : Shape, scale : Vec3, transform : Mat, tmpHit : HitResult, collector : RayCastCollector ) : Bool {
		var shape : ScaledShape = hl.Api.unsafeCast(shape);
		var tmpScale = freeVec.get();
		tmpScale.load(scale.multiplied(shape.scale));
		var b = algoCtx.raycast(ray, shape.shape, tmpScale, transform, tmpHit, collector);
		freeVec.put(tmpScale);
		return b;

	}
}
