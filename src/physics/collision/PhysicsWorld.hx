package physics.collision;

typedef BodyID = ID;

private class BroadphaseVisitor extends TreeVisitor {
	var b1 : Int;
	var aabb : AABB;
	var processPair : (BodyID, BodyID, Int) -> Void;
	public var workerID : Int;

	public function new() {
	}

	public function init( node : TreeNode, processPair : (BodyID, BodyID, Int) -> Void, workerID : Int ) {
		b1 = node.bodyID;
		aabb = node.aabb;
		this.processPair = processPair;
		this.workerID = workerID;
	}

	public function visitNode( node : TreeNode ) {
		return aabb.collide(node.aabb);
	}

	public function visitBody( body : TreeNode ) {
		if ( aabb.collide(body.aabb) )
			processPair(b1, body.bodyID, workerID);
		return true;
	}
}

private class WorkerData {
	public var collector(default, null) : CollideCollector;
	public var broadphaseVisitor : BroadphaseVisitor;
	public var algoCtx : AlgorithmContext;

	public function new( bodyCount : Int ) {
		collector = new CollideCollector();
		broadphaseVisitor = new BroadphaseVisitor();
		algoCtx = new AlgorithmContext();
	}

	public function reset() {
		collector.init(Math.SCALAR_MAX, All);
	}
}

private class CollideVisitor extends TreeVisitor {
	var shape : Shape;
	var scale : Vec3;
	var transform : Mat;
	var world : PhysicsWorld;
	var collector : CollideCollector;
	var collisionMask : Int;
	var algoCtx : AlgorithmContext;
	@:packed var shapeAABB : AABB;

	public function new() {
		this.algoCtx = new AlgorithmContext();
	}

	public function init( shape : Shape, scale : Vec3, transform : Mat, world : PhysicsWorld, collector : CollideCollector, collisionMask : Int ) {
		this.shape = shape;
		this.scale = scale;
		this.transform = transform;
		this.world = world;
		this.collector = collector;
		this.collisionMask = collisionMask;
		shape.getLocalBoundsToBuffer(this.shapeAABB);
		this.shapeAABB.transform(transform);
	}

	public function visitNode( node : TreeNode ) : Bool {
		return node.aabb.collide(shapeAABB);
	}

	public function visitBody( node : TreeNode ) : Bool {
		var id = node.bodyID;
		var body = world.getBody(id);
		if( body.filter(collisionMask) && node.aabb.collide(shapeAABB) ) {
			collector.onBody(id);
			algoCtx.collide(shape, body.shape, scale, body.scale, transform, body.transform, collector);
			collector.onBodyEnd();
		}
		return collector.curWantsMoreHits;
	}
}

private class ShapeCastVisitor extends TreeVisitor {
	var shapeCast : ShapeCast;
	var collisionMask : Int;
	var collector : ShapeCastCollector;
	var world : PhysicsWorld;
	var algoCtx : AlgorithmContext;
	@:packed var ray : Ray;
	@:packed var halfExtent : Vec3;
	@:packed var aabb : AABB;

	public function new() {
		this.algoCtx = new AlgorithmContext();
	}

	public function init( shapeCast : ShapeCast, world : PhysicsWorld, collector : ShapeCastCollector, collisionMask : Int ) {
		this.shapeCast = shapeCast;
		this.collisionMask = collisionMask;
		this.collector = collector;
		this.world = world;
		shapeCast.shape.getLocalBoundsToBuffer(aabb);
		aabb.transform(shapeCast.transform);
		this.ray.direction.load(shapeCast.direction);
		this.ray.origin.load(aabb.getCenter());
		this.halfExtent.load(aabb.getExtent().scaled(0.5));
	}

	public function visitNode( node : TreeNode ) : Bool {
		var enlargeAABB = node.aabb.clone();
		enlargeAABB.enlargeWithExtent(halfExtent);
		return enlargeAABB.raycast(ray) < collector.curMaxFraction;
	}

	public function visitBody( node : TreeNode ) : Bool {
		var enlargeAABB = node.aabb.clone();
		enlargeAABB.enlargeWithExtent(halfExtent);
		var id = node.bodyID;
		var body = world.getBody(id);
		if ( body.filter(collisionMask) && enlargeAABB.raycast(ray) < collector.curMaxFraction ) {
			collector.onBody(id);
			algoCtx.shapecast(shapeCast, body.shape, body.scale, body.transform, collector);
			collector.onBodyEnd();
		}
		return collector.curWantsMoreHits;
	}
}

private class RayCastVisitor extends TreeVisitor {
	var ray : Ray;
	var world : PhysicsWorld;
	var collector : RayCastCollector;
	var collisionMask : Int;
	var algoCtx : AlgorithmContext;
	@:packed var hitResult : HitResult;

	public function new() {
		this.algoCtx = new AlgorithmContext();
	}

	public function init( ray : Ray, world : PhysicsWorld, collector : RayCastCollector, collisionMask : Int ) {
		this.ray = ray;
		this.world = world;
		this.collector = collector;
		this.collisionMask = collisionMask;
	}

	public function visitNode( node : TreeNode ) : Bool {
		return node.aabb.raycast(ray) < collector.curMaxFraction;
	}

	public function visitBody( node : TreeNode ) : Bool {
		var id = node.bodyID;
		var body = world.getBody(id);
		if ( body.filter(collisionMask) && node.aabb.raycast(ray) < collector.curMaxFraction ) {
			collector.onBody(id);
			algoCtx.raycast(ray, body.shape, body.scale, body.transform, hitResult, collector);
			collector.onBodyEnd();
		}
		return collector.curWantsMoreHits;
	}
}

class PhysicsWorld {
	var bodies : PhysicsContainer<Body>;
	var tree : AABBTree;
	var jobSystem : JobSystem;

	var workersData : hl.NativeArray<WorkerData>;

	var maxBodiesPerWorker(get, never) : Int ;
	function get_maxBodiesPerWorker() {
		return Math.floor(bodies.capacity / (workerCount + 1));
	}

	var bodiesPerWorker(get, never) : Int ;
	function get_bodiesPerWorker() {
		return Math.floor(bodies.length / (workerCount + 1));
	}

	public var workerCount(default, set) : Int = 4;
	public function set_workerCount( v : Int ) {
		if ( v != workerCount && enableThreading ) {
			jobSystem.shutdown();
			jobSystem = new JobSystem(v);
			workerCount = v;
			initWorkerData();
		}
		return workerCount;
	}
	public var enableThreading(default, set) : Bool = false;
	public function set_enableThreading( v : Bool ) {
		if ( v != enableThreading ) {
			if ( v )
				jobSystem = new JobSystem(workerCount);
			else {
				jobSystem.shutdown();
				jobSystem = null;
			}
			enableThreading = v;
			initWorkerData();
		}
		return enableThreading;
	}

	public var profiler : Profiler;
	var collideVisitor : CollideVisitor;
	var collideCollector : CollideCollector;
	var shapeCastVisitor : ShapeCastVisitor;
	var shapeCastCollector : ShapeCastCollector;
	var rayCastVisitor : RayCastVisitor;
	var rayCastCollector : RayCastCollector;

	public function new( initialCapacity : Int, profiler : Profiler = null ) {
		this.profiler = profiler;
		bodies = new PhysicsContainer(Body, initialCapacity);
		tree = new AABBTree(0.);
		initWorkerData();
	}

	public function addBody( bodyToAdd : Body ) : BodyID {
		var id = bodies.create();
		var body = bodies.get(id);
		body.load(bodyToAdd);
		var aabb = body.getWorldBounds();
		body.nodeID = tree.addBody(aabb, id);
		return id;
	}

	public inline function getBody( id : BodyID ) {
		return bodies.get(id);
	}

	public inline function removeBody( id : BodyID ) {
		var body = bodies.get(id);
		tree.removeBody(body.nodeID);
		body.userData = null;
		body.shape = null;
		bodies.remove(id);
	}

	public inline function collide( shape : Shape, scale : Vec3, transform : Mat, callback : (ContactPoint, BodyID) -> Bool, collisionMask : Int = ~0, mode : CollectorMode = All ) {
		if( collideVisitor == null ) {
			collideVisitor = new CollideVisitor();
			collideCollector = new CollideCollector();
		}
		Assert.t(shape.isScaleValid(scale));
		collideCollector.init(Math.SCALAR_MAX, mode);
		collideVisitor.init(shape, scale, transform, this, collideCollector, collisionMask);
		tree.walkTree(collideVisitor);
		collideCollector.iterResult(callback);
	}

	public inline function shapecast( shapeCast : ShapeCast, callback : (ShapeCastResult, BodyID) -> Bool, maxFraction : Scalar = Math.SCALAR_MAX, collisionMask : Int = ~0, mode : CollectorMode = Closest ) {
		if( shapeCastVisitor == null ) {
			shapeCastVisitor = new ShapeCastVisitor();
			shapeCastCollector = new ShapeCastCollector();
		}
		shapeCastCollector.init(maxFraction, mode);
		shapeCastVisitor.init(shapeCast, this, shapeCastCollector, collisionMask);
		tree.walkTree(shapeCastVisitor);
		shapeCastCollector.iterResult(callback);
	}

	public inline function raycast( ray : Ray, callback : (HitResult, BodyID) -> Bool, maxFraction : Scalar = Math.SCALAR_MAX, collisionMask : Int = ~0, mode : CollectorMode = Closest ) {
		if( rayCastVisitor == null ) {
			rayCastVisitor = new RayCastVisitor();
			rayCastCollector = new RayCastCollector();
		}
		rayCastCollector.init(maxFraction, mode);
		rayCastVisitor.init(ray, this, rayCastCollector, collisionMask);
		tree.walkTree(rayCastVisitor);
		rayCastCollector.iterResult(callback);
	}

	public function update() {
		mark("Update bodies");
		for ( b in bodies ) {
			if ( b.motionType == Static )
				continue;
			var aabb = b.getWorldBounds();
			tree.updateBody(b.nodeID, aabb, false);
		}
	}

	public function findCollisionsByPair( callback : (arr:StaticArray<ContactPoint>, start:Int, count:Int, b1:Int, b2:Int) -> Void ) {
		findCollisions();
		for ( w in workersData )
			w.collector.iterResultsByPair(callback);
	}

	inline function mark( name : String ) {
		#if !release
		if ( profiler != null )
			profiler.mark(name);
		#end
	}

	inline function initWorkerData() {
		var bodyCount = bodies.capacity;
		if ( !enableThreading ) {
			workersData = new hl.NativeArray(1);
			workersData[0] = new WorkerData(bodyCount);
		} else {
			workersData = new hl.NativeArray(workerCount + 1);
			var maxBodiesPerWorker = maxBodiesPerWorker;
			for ( i in 0...workerCount )
				workersData[i] = new WorkerData(maxBodiesPerWorker);
			workersData[workerCount] = new WorkerData(bodyCount - maxBodiesPerWorker * workerCount);
		}
	}

	function jobFindCollisions( workerID : Int, start : Int, end : Int ) {
		var visitor = workersData[workerID].broadphaseVisitor;
		for ( i in start...end ) {
			var b = bodies.at(i);
			if ( b.motionType != Dynamic )
				continue;
			var collector = workersData[workerID].collector;
			collector.setBody1(i);
			visitor.init(tree.getTreeNode(b.nodeID), processPair, workerID);
			tree.walkTree(visitor);
		}
	}

	function findCollisions() {
		mark("FindCollisions");
		for ( w in workersData )
			w.reset();
		if ( enableThreading ) {
			var bodiesPerWorker = bodiesPerWorker;
			for ( w in 0...workerCount )
				jobSystem.runJob( (wid) -> jobFindCollisions(wid, bodiesPerWorker * wid, bodiesPerWorker * (wid + 1)) );
			jobFindCollisions(workerCount, bodiesPerWorker * workerCount, bodies.length);
			jobSystem.waitForJob();
		} else {
			jobFindCollisions(0, 0, bodies.length);
		}
	}

	function processPair( b1 : Int, b2 : Int, workerID : Int ) {
		var collector = workersData[workerID].collector;
		var algoCtx = workersData[workerID].algoCtx;
		var body1 = bodies.get(b1);
		var body2 = bodies.get(b2);
		var shouldCollide = ( b1 > b2 || body2.motionType != Dynamic ) && body1.filter(body2.collisionMask) && body2.filter(body1.collisionMask);
		if ( shouldCollide ) {
			collector.onBody(b2);
			algoCtx.collide(body1.shape, body2.shape, body1.scale, body2.scale, body1.transform, body2.transform, collector);
			collector.onBodyEnd();
		}
	}

	#if physics_profile
	public function startProfile() {
		collideCollector?.profileStart(this);
		shapeCastCollector?.profileStart(this);
		rayCastCollector?.profileStart(this);
	}

	public function stopProfile(output: String) {
		var lines : Array<String> = [];
		function section( name : String, map : Null<Map<Shape, Float>> ) {
			if( map == null ) return;
			var sorted = [for( shape => time in map ) { shape: shape, time: time }];
			sorted.sort((a, b) -> a.time < b.time ? 1 : a.time > b.time ? -1 : 0);
			var total = 0.0;
			for( e in sorted ) total += e.time;
			lines.push('[$name] ${hxd.Math.fmt(total*1000.0)} ms');
			for( e in sorted ) {
				var id = e.shape.toString();
				if(e.shape.profileTag != null)
					id = '${e.shape.profileTag}: $id';
				var pc = total > 0 ? hxd.Math.fmt(100.0 * e.time / total) : 0.0;
				lines.push('$pc% ; $id');
			}
			lines.push("\n\n");
		}
		section("Collide", collideCollector?.profileStop());
		section("ShapeCast", shapeCastCollector?.profileStop());
		section("RayCast", rayCastCollector?.profileStop());
		sys.io.File.saveContent(output, lines.join("\n"));
	}
	#end
}
