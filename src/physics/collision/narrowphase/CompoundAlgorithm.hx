package physics.collision.narrowphase;

class CompoundAlgorithm {

	var algoCtx : AlgorithmContext;
	var freeVec : Pool<Vec3Impl>;
	var freeMat : Pool<MatImpl>;
	var freeShapecast : Pool<ShapeCast>;

	public function new( algoCtx : AlgorithmContext ) {
		this.algoCtx = algoCtx;
		this.freeVec = new Pool(Vec3Impl, () -> Vec3.zero());
		this.freeMat = new Pool(MatImpl, () -> new Mat());
		this.freeShapecast = new Pool(ShapeCast, () -> new ShapeCast());
	}

	public function collideShapeVsCompound( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) : Bool {
		var shape2 : CompoundShape = hl.Api.unsafeCast(shape2);

		var hasCollide = false;
		for( sub in shape2.subShapes ) {
			var subScale = freeVec.get();
			subScale.load(sub.transformScale(scale2));
			var subTrans = freeMat.get();
			subTrans.multiply(sub.getTransformMatrix(scale2), transform2);
			if( algoCtx.collide(shape1, sub.shape, scale1, subScale, transform1, subTrans, collector) ) {
				hasCollide = true;
			}
			freeMat.put(subTrans);
			freeVec.put(subScale);
		}
		return hasCollide;
	}

	public function collideCompoundVsShape( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) : Bool {
		var shape1 : CompoundShape = hl.Api.unsafeCast(shape1);

		var hasCollide = false;
		for( sub in shape1.subShapes ) {
			var subScale = freeVec.get();
			subScale.load(sub.transformScale(scale1));
			var subTrans = freeMat.get();
			subTrans.multiply(sub.getTransformMatrix(scale1), transform1);
			if( algoCtx.collide(sub.shape, shape2, subScale, scale2, subTrans, transform2, collector) ) {
				hasCollide = true;
			}
			freeMat.put(subTrans);
			freeVec.put(subScale);
		}
		return hasCollide;
	}

	public function shapecastShapeVsCompound( shapeCast : ShapeCast, shape2 : Shape, scale2 : Vec3, transform2 : Mat, collector : ShapeCastCollector ) : Bool {
		var shape2 : CompoundShape = hl.Api.unsafeCast(shape2);

		var hasCollide = false;
		for( sub in shape2.subShapes ) {
			var subScale = freeVec.get();
			subScale.load(sub.transformScale(scale2));
			var subTrans = freeMat.get();
			subTrans.multiply(sub.getTransformMatrix(scale2), transform2);
			if( algoCtx.shapecast(shapeCast, sub.shape, subScale, subTrans, collector) ) {
				hasCollide = true;
			}
			freeMat.put(subTrans);
			freeVec.put(subScale);
		}
		return hasCollide;
	}

	public function shapecastCompoundVsShape( shapeCast : ShapeCast, shape2 : Shape, scale2 : Vec3, transform2 : Mat, collector : ShapeCastCollector ) : Bool {
		var shape1 : CompoundShape = hl.Api.unsafeCast(shapeCast.shape);
		if( shape1.mustBeStatic() )
			throw "Can't shapecast from " + shape1;

		var hasCollide = false;
		for( sub in shape1.subShapes ) {
			var subShapeCast = freeShapecast.get();
			subShapeCast.shape = sub.shape;
			subShapeCast.direction = shapeCast.direction;
			subShapeCast.transform = sub.getTransformMatrix(shapeCast.scale).multiplied(shapeCast.transform);
			subShapeCast.scale = sub.transformScale(shapeCast.scale);
			if( algoCtx.shapecast(subShapeCast, shape2, scale2, transform2, collector) ) {
				hasCollide = true;
			}
			freeShapecast.put(subShapeCast);
		}
		return hasCollide;
	}

	public function raycast( ray : Ray, shape : Shape, scale : Vec3, transform : Mat, tmpHit : HitResult, collector : RayCastCollector ) : Bool {
		var shape : CompoundShape = hl.Api.unsafeCast(shape);

		var hasCollide = false;
		for( sub in shape.subShapes ) {
			var subScale = freeVec.get();
			subScale.load(sub.transformScale(scale));
			var subTrans = freeMat.get();
			subTrans.multiply(sub.getTransformMatrix(scale), transform);
			if( algoCtx.raycast(ray, sub.shape, subScale, subTrans, tmpHit, collector) ) {
				hasCollide = true;
			}
			freeMat.put(subTrans);
			freeVec.put(subScale);
		}
		return hasCollide;
	}
}
