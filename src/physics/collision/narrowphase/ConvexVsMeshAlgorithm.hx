package physics.collision.narrowphase;

class MeshCollideVisitor extends TreeVisitor {
	var convex : ConvexShape;
	var mesh : MeshShape;
	var scaleConvex : Vec3;
	var scaleMesh : Vec3;
	var transformConvex : Mat;
	var transformMesh : Mat;
	var collector : CollideCollector;
	var algo : ConvexVsConvexAlgorithm;
	@:packed var transformMeshToConvex : Mat;
	@:packed var convexAABBInSpaceOfMesh : AABB;

	public function new() {
		this.algo = new ConvexVsConvexAlgorithm();
	}

	public function init( convex : ConvexShape, mesh : MeshShape, scaleConvex : Vec3, scaleMesh : Vec3, transformConvex : Mat, transformMesh : Mat, collector : CollideCollector ) {
		this.convex = convex;
		this.mesh = mesh;
		this.scaleConvex = scaleConvex;
		this.scaleMesh = scaleMesh;
		this.transformConvex = transformConvex;
		this.transformMesh = transformMesh;
		this.collector = collector;
		this.transformMeshToConvex.load(transformMesh.multiplied(transformConvex.getInverse()));
		convex.getLocalBoundsToBuffer(this.convexAABBInSpaceOfMesh);
		this.convexAABBInSpaceOfMesh.scale(scaleConvex);
		this.convexAABBInSpaceOfMesh.transform(transformMeshToConvex.getInverse());
	}

	public function visitNode( node : TreeNode ) : Bool {
		return node.aabb.scaled(scaleMesh).collide(convexAABBInSpaceOfMesh);
	}

	public function visitBody( node : TreeNode ) : Bool {
		if( node.aabb.scaled(scaleMesh).collide(convexAABBInSpaceOfMesh) ) {
			var id = node.bodyID;
			var tri = @:privateAccess mesh.getTriangle(id);
			algo.testCollision(convex, tri, scaleConvex, scaleMesh, transformConvex, transformMesh, collector);
		}
		return collector.curWantsMoreHits;
	}
}

class MeshShapeCastVisitor extends TreeVisitor {
	var convex : ShapeCast;
	var mesh : MeshShape;
	var scaleMesh : Vec3;
	var transformMesh : Mat;
	var collector : ShapeCastCollector;
	var algo : ConvexVsConvexAlgorithm;
	@:packed var aabb : AABB;
	@:packed var halfExtent : Vec3;
	@:packed var ray : Ray;

	public function new() {
		this.algo = new ConvexVsConvexAlgorithm();
	}

	public function init( convex : ShapeCast, mesh : MeshShape, scaleMesh : Vec3, transformMesh : Mat, collector : ShapeCastCollector ) {
		this.convex = convex;
		this.mesh = mesh;
		this.scaleMesh = scaleMesh;
		this.transformMesh = transformMesh;
		this.collector = collector;
		var invTransformMesh = transformMesh.getInverse();
		var body1ToBody2 = convex.transform.multiplied(invTransformMesh);
		convex.shape.getLocalBoundsToBuffer(aabb);
		aabb.scale(convex.scale);
		aabb.transform(body1ToBody2);
		this.halfExtent.load(aabb.getExtent().scaled(0.5));
		this.ray.direction.load(convex.direction.transformed3x3(invTransformMesh));
		this.ray.origin.load(aabb.getCenter());
	}

	public function visitNode( node : TreeNode ) : Bool {
		var enlargeAABB = node.aabb.scaled(scaleMesh);
		enlargeAABB.enlargeWithExtent(halfExtent);
		return enlargeAABB.raycast(ray) < collector.curMaxFraction;
	}

	public function visitBody( node : TreeNode ) : Bool {
		var enlargeAABB = node.aabb.scaled(scaleMesh);
		enlargeAABB.enlargeWithExtent(halfExtent);
		if( enlargeAABB.raycast(ray) < collector.curMaxFraction ) {
			var id = node.bodyID;
			var tri = @:privateAccess mesh.getTriangle(id);
			algo.shapecast(convex, tri, scaleMesh, transformMesh, collector);
		}
		return collector.curWantsMoreHits;
	}
}

class MeshRayCastVisitor extends TreeVisitor {
	@:packed var ray : Ray;
	var mesh : MeshShape;
	var scaleMesh : Vec3;
	public var collector : RayCastCollector;
	@:packed var hitResult : HitResult;

	public function new() {
		this.collector = new RayCastCollector();
	}

	public inline function init( rayOrigin : Vec3, rayDirection : Vec3, mesh : MeshShape, scaleMesh : Vec3, maxFraction : Scalar, mode : CollectorMode ) {
		this.ray.origin.load(rayOrigin);
		this.ray.direction.load(rayDirection);
		this.mesh = mesh;
		this.scaleMesh = scaleMesh;
		this.collector.init(maxFraction, mode);
	}

	public function visitNode( node : TreeNode ) : Bool {
		return node.aabb.scaled(scaleMesh).raycast(ray) < collector.curMaxFraction;
	}

	public function visitBody( node : TreeNode ) : Bool {
		if ( node.aabb.scaled(scaleMesh).raycast(ray) < collector.curMaxFraction ) {
			var id = node.bodyID;
			var tri = @:privateAccess mesh.getTriangle(id).scaled(scaleMesh);
			if ( tri.raycast(ray, Vec3.one(), Mat.identity(), hitResult) ) {
				collector.onBody(id);
				collector.addHit(hitResult.position, hitResult.normal, hitResult.fraction);
				collector.onBodyEnd();
			}
		}
		return collector.curWantsMoreHits;
	}
}

class ConvexVsMeshAlgorithm {
	var meshCollideVisitor : MeshCollideVisitor;
	var meshShapeCastVisitor : MeshShapeCastVisitor;
	var meshRayCastVisitor : MeshRayCastVisitor;
	var tmpNode : TreeNode;

	public function new() {
	}

	public function testCollision( shape1 : Shape, shape2 : Shape, scale1 : Vec3, scale2 : Vec3, transform1 : Mat, transform2 : Mat, collector : CollideCollector ) {
		var shape1 : ConvexShape = hl.Api.unsafeCast(shape1);
		var shape2 : MeshShape = hl.Api.unsafeCast(shape2);

		if( meshCollideVisitor == null )
			meshCollideVisitor = new MeshCollideVisitor();
		if( tmpNode == null )
			tmpNode = new TreeNode();
		var start = collector.length;
		meshCollideVisitor.init(shape1, shape2, scale1, scale2, transform1, transform2, collector);
		shape2.walk(meshCollideVisitor, tmpNode);
		return start < collector.length;
	}

	public function shapecast( shapeCast : ShapeCast, shape2 : Shape, scale2 : Vec3, transform2 : Mat, collector : ShapeCastCollector ) : Bool {
		var shape1 : ConvexShape = hl.Api.unsafeCast(shapeCast.shape);
		var shape2 : MeshShape = hl.Api.unsafeCast(shape2);

		if( meshShapeCastVisitor == null )
			meshShapeCastVisitor = new MeshShapeCastVisitor();
		if( tmpNode == null )
			tmpNode = new TreeNode();
		var start = collector.length;
		meshShapeCastVisitor.init(shapeCast, shape2, scale2, transform2, collector);
		shape2.walk(meshShapeCastVisitor, tmpNode);
		return start < collector.length;
	}

	public function raycast( ray : Ray, shape : Shape, scale : Vec3, transform : Mat, tmpHit : HitResult, collector : RayCastCollector ) : Bool {
		var shape : MeshShape = hl.Api.unsafeCast(shape);

		var invTransform = transform.getInverse();
		var torigin = ray.origin.transformed(invTransform);
		var tdirection = ray.direction.transformed3x3(invTransform);
		if( meshRayCastVisitor == null )
			meshRayCastVisitor = new MeshRayCastVisitor();
		if( tmpNode == null )
			tmpNode = new TreeNode();
		var maxFraction = collector.curMaxFraction;
		var mode = collector.mode == ClosestPerBody ? Closest : collector.mode;
		meshRayCastVisitor.init(torigin, tdirection, shape, scale, maxFraction, mode);
		shape.walk(meshRayCastVisitor, tmpNode);
		meshRayCastVisitor.collector.iterResult(function(best, bestId) {
			var position = ray.getPoint(best.fraction);
			var normal = shape.getTriangle(bestId).getSurfaceNormal(Vec3.zero()).transformed3x3(transform);
			collector.addHit(position, normal, best.fraction);
		});
		return meshRayCastVisitor.collector.hasResult();
	}
}
