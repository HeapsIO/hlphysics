inline final DEFAULT_GROUP = (1 << 0);
inline final OBSTACLE_GROUP = (1 << 1);
inline final CHARACTER_GROUP = (1 << 2);

abstract class Rigidbody extends h3d.scene.Object {

	public var bodyID(default, null) : BodyID;
	var world : PhysicsWorld;

	abstract public function makeShape() : Shape;

	abstract public function makeObject( ?material : h3d.mat.Material, ?parent : h3d.scene.Object ) : h3d.scene.Object;

	override function sync(ctx) {
		if ( world != null ) {
			var body = world.getBody(bodyID);
			x = body.position.x;
			y = body.position.y;
			z = body.position.z;
		}
		super.sync(ctx);
	}

	public function setRot( quat : h3d.Quat ) {
		qRot.load(quat);
		if ( world != null ) {
			var body = world.getBody(bodyID);
			body.rotation.value.x = quat.x;
			body.rotation.value.y = quat.y;
			body.rotation.value.z = quat.z;
			body.rotation.value.w = quat.w;
		}
	}

	public function setPos( vec : h3d.Vector ) {
		x = vec.x;
		y = vec.y;
		z = vec.z;
		if ( world != null ) {
			var body = world.getBody(bodyID);
			body.position.x = vec.x;
			body.position.y = vec.y;
			body.position.z = vec.z;
		}
	}

	override function onRemove() {
		super.onRemove();
		if ( world != null )
			world.removeBody(bodyID);
		world = null;
	}

	public function createBody( world : PhysicsWorld, motionType : MotionType = Static, collisionGroup : Int = DEFAULT_GROUP ) {
		this.world = world;

		var b = new Body(makeShape());
		b.collisionGroup = collisionGroup;

		var rot = getAbsPos().getEulerAngles();
		b.setPosition(x,y,z);
		b.setRotation(rot.x, rot.y, rot.z);
		b.setScale(scaleX, scaleY, scaleZ);
		b.setMotionType(motionType);

		bodyID = world.addBody(b);
	}

}

class Cylinder extends Rigidbody {
	var obj : h3d.scene.Object;
	var cylinderPrim : h3d.prim.Cylinder;
	var discPrim : h3d.prim.Disc;
	var top : h3d.scene.Mesh;
	var bottom : h3d.scene.Mesh;
	var radius: Float;
	var height : Float;

	public function new( radius : Float, height : Float, ?material, ?parent ) {
		super(parent);
		this.radius = radius;
		this.height = height;
		obj = makeObject(material, this);
	}

	public function makeShape() {
		// var a = new h3d.Vector(0, 0, height/2);
		// var b = new h3d.Vector(0, 0, -height/2);
		// var col = new h3d.col.Cylinder(a, b, radius);
		// return Shape.fromHeaps(col);
		return new CylinderShape(radius, height/2);
	}

	public function makeObject( ?material, ?parent ) : h3d.scene.Object {
		var obj = new h3d.scene.Object(parent);

		if (cylinderPrim == null ) {
			cylinderPrim = new h3d.prim.Cylinder(16, radius, height, true);
			cylinderPrim.addNormals();
			cylinderPrim.addUVs();
		}

		if ( discPrim == null ) {
			discPrim = new h3d.prim.Disc(radius, 16);
			discPrim.addNormals();
			discPrim.addUVs();
		}

		var cylinder = new h3d.scene.Mesh( cylinderPrim, material, obj);
		var top = new h3d.scene.Mesh(discPrim, material, obj);
		top.z = height * 0.5;
		var bottom = new h3d.scene.Mesh(discPrim, material, obj);
		bottom.rotate(hxd.Math.PI, 0.0, 0.0);
		bottom.z = height * -0.5;

		return obj;
	}
}

class Capsule extends Rigidbody {
	var mesh : h3d.scene.Object;
	var prim : h3d.prim.Capsule;
	var radius: Float;
	var height : Float;

	public function new( radius : Float, height : Float, ?material, ?parent ) {
		super(parent);
		this.radius = radius;
		this.height = height;
		mesh = makeObject(material, this);
	}

	public function makeShape() {
		// var a = new h3d.Vector(0, 0, height/2);
		// var b = new h3d.Vector(0, 0, -height/2);
		// var col = new h3d.col.Capsule(a, b, radius);
		// return Shape.fromHeaps(col);
		return new CapsuleShape(radius, height/2);
	}

	public function makeObject( ?material, ?parent ) : h3d.scene.Object {
		if ( prim == null ) {
			prim = new h3d.prim.Capsule(radius, height);
			prim.addNormals();
		}

		var root = new h3d.scene.Object(parent);
		var obj = new h3d.scene.Mesh(prim, material, root);
		obj.rotate(0.0, hxd.Math.PI*0.5, 0.0);
		return root;
	}
}

class Hull extends Rigidbody {
	var col : h3d.col.PolygonBuffer;
	var mesh : h3d.scene.Object;
	var points : Array<Scalar>;
	var indices : Array<Int>;
	var modelPath : String;

	public function new( modelPath : String, ?material, ?parent ) {
		super(parent);
		this.modelPath = modelPath;
		mesh = makeObject(material, this);
		var meshes = mesh.getMeshes();
		for ( m in meshes ) {
			var prim = Std.downcast(m.primitive, h3d.prim.HMDModel);
			var collider : h3d.col.Collider.OptimizedCollider = cast prim.getCollider();
			col = Std.downcast(collider.b, h3d.col.PolygonBuffer);
			if ( col == null ) {
				var collider : h3d.col.Collider.GroupCollider = cast collider.b;
				col = cast collider.colliders[0];
			}
			break;
		}
		points = [for ( i in @:privateAccess col.buffer) i];
		indices = [for ( i in @:privateAccess col.indexes) i];
	}

	public function makeShape() {
		var s = new ConvexHullShape( points, indices );
		return s;
	}

	public function makeObject( ?material:h3d.mat.Material, ?parent:h3d.scene.Object ) : h3d.scene.Object {
		var cache = new h3d.prim.ModelCache();
		var mesh = cache.loadModel(hxd.Res.load(modelPath).toModel());
		var meshes = mesh.getMeshes();
		if ( material != null )
			for ( m in meshes )
				m.material = material;
		parent.addChild(mesh);
		cache.dispose();
		return mesh;
	}
}

class HullSimple extends Rigidbody {
	var mesh : h3d.scene.Object;
	var points : Array<Scalar>;
	var indices : Array<Int>;
	var prim : h3d.prim.Polygon;
	var col : h3d.col.Collider;
	var varray : Array<Vec3>;

	public function new( size : Float, thickness : Float, ?material, ?parent ) {
		super(parent);
		varray = [
			new Vec3(-size,-size,thickness), new Vec3(size,-size,thickness), new Vec3(size,size,thickness), new Vec3(-size,size,thickness),
			new Vec3(-size,-size,0), new Vec3(size,-size,0), new Vec3(size,size,0), new Vec3(-size,size,0),
		];
		points = [
			-size,-size,0,size,-size,0,size,size,0,-size,size,0,
			-size,-size,thickness,size,-size,thickness,size,size,thickness,-size,size,thickness,
		];
		indices = [
			2,3,0,0,1,2,0,4,5,0,5,1,1,5,6,1,6,2,
			2,6,7,2,7,3,3,7,4,3,4,0,6,5,4,4,7,6,
		];
		mesh = makeObject(material, this);
	}

	public function makeShape() {
		return new ConvexHullShape(points, indices);
	}

	public function makeObject( ?material, ?parent ) : h3d.scene.Object {
		if ( prim == null ) {
			var idx = new hxd.IndexBuffer();
			for( i in indices ) {
				idx.push(i);
			}
			var points = varray.map(v -> v.toHeaps());
			prim = new h3d.prim.Polygon(points, idx);
			prim.addNormals();
		}
		var mesh = new h3d.scene.Mesh(prim, material, parent);
		mesh.material.mainPass.culling = None;
		return mesh;
	}
}

class Mesh extends Rigidbody {
	var col : h3d.col.PolygonBuffer;
	var mesh : h3d.scene.Object;
	var points : Array<Scalar>;
	var indices : Array<Int>;
	var modelPath : String;

	public function new( modelPath : String, ?material, ?parent ) {
		super(parent);
		this.modelPath = modelPath;
		mesh = makeObject(material, this);
		var meshes = mesh.getMeshes();
		for ( m in meshes ) {
			var prim = Std.downcast(m.primitive, h3d.prim.HMDModel);
			var collider : h3d.col.Collider.OptimizedCollider = cast prim.getCollider();
			col = Std.downcast(collider.b, h3d.col.PolygonBuffer);
			if ( col == null ) {
				var collider : h3d.col.Collider.GroupCollider = cast collider.b;
				col = cast collider.colliders[0];
			}
			break;
		}
		points = [for ( i in @:privateAccess col.buffer) i];
		indices = [for ( i in @:privateAccess col.indexes) i];
	}

	public function makeShape() {
		var s = new MeshShape( points, indices );
		return s;
	}

	public function makeObject( ?material:h3d.mat.Material, ?parent:h3d.scene.Object ) : h3d.scene.Object {
		var cache = new h3d.prim.ModelCache();
		var mesh = cache.loadModel(hxd.Res.load(modelPath).toModel());
		var meshes = mesh.getMeshes();
		if ( material != null )
			for ( m in meshes )
				m.material = material;
		parent.addChild(mesh);
		cache.dispose();
		return mesh;
	}
}

class MeshSimple extends Rigidbody {
	var mesh : h3d.scene.Object;
	var points : Array<Scalar>;
	var indices : Array<Int>;
	var prim : h3d.prim.Polygon;
	var col : h3d.col.Collider;
	var varray : Array<Vec3>;

	public function new(?material, ?parent ) {
		super(parent);
		var size = 2.0;
		varray = [new Vec3(0,0,0), new Vec3(size,0,0), new Vec3(size,size,0), new Vec3(0,size,0), new Vec3(0,0,size)];
		points = [0,0,0,size,0,0,size,size,0,0,size,0,0,0,size];
		indices = [0,1,2,0,2,3,0,1,4];
		mesh = makeObject(material, this);
	}

	public function makeShape() {
		return new MeshShape(points, indices);
	}

	public function makeObject( ?material, ?parent ) : h3d.scene.Object {
		if ( prim == null ) {
			var idx = new hxd.IndexBuffer();
			for( i in indices ) {
				idx.push(i);
			}
			var points = varray.map(v -> v.toHeaps());
			prim = new h3d.prim.Polygon(points, idx);
			prim.addNormals();
		}
		var mesh = new h3d.scene.Mesh(prim, material, parent);
		mesh.material.mainPass.culling = None;
		return mesh;
	}
}

class Model extends Rigidbody {
	var mesh : h3d.scene.Object;
	var modelPath : String;

	public function new( modelPath : String, ?material, ?parent ) {
		super(parent);
		this.modelPath = modelPath;
		mesh = makeObject(material, this);
	}

	public function makeShape() {
		var col = Shape.fromHeaps(mesh.getCollider());
		var mat = mesh.defaultTransform;
		if( mat != null && !mat.isIdentity() ) {
			col = Shape.transformed(col, Vec3.fromHeaps(mat.getPosition()), Vec3.fromHeaps(mat.getEulerAngles()), Vec3.fromHeaps(mat.getScale()));
		}
		return col;
	}

	public function makeObject( ?material:h3d.mat.Material, ?parent:h3d.scene.Object ) : h3d.scene.Object {
		var cache = new h3d.prim.ModelCache();
		var mesh = cache.loadModel(hxd.Res.load(modelPath).toModel());
		var meshes = mesh.getMeshes();
		if ( material != null )
			for ( m in meshes )
				m.material = material;
		parent.addChild(mesh);
		cache.dispose();
		return mesh;
	}
}

class Box extends Rigidbody {
	var mesh : h3d.scene.Object;
	var size : h3d.Vector;

	public function new( size : h3d.Vector, ?material, ?parent ) {
		super(parent);
		this.size = size;
		mesh = makeObject(material, this);
	}

	public function makeShape() {
		// var col = new h3d.col.OrientedBounds();
		// var m = h3d.Matrix.S(size.x, size.y, size.z);
		// col.setMatrix(m);
		// return Shape.fromHeaps(col);
		return new BoxShape(new Vec3(size.x * 0.5, size.y * 0.5, size.z * 0.5));
	}

	public function makeObject( ?material, ?parent ) : h3d.scene.Object {
		var obj = new h3d.scene.Mesh(h3d.prim.Cube.defaultUnitCube(), material, parent);
		obj.scaleX = size.x;
		obj.scaleY = size.y;
		obj.scaleZ = size.z;
		return obj;
	}
}

class Sphere extends Rigidbody {
	var mesh : h3d.scene.Object;
	var radius: Float;
	var prim : h3d.prim.Sphere;

	public function new(radius : Float, ?material, ?parent ) {
		super(parent);
		this.radius = radius;
		mesh = makeObject(material, this);
	}

	public function makeShape() {
		// var col = new h3d.col.Sphere(0.0, 0.0, 0.0, radius);
		// return Shape.fromHeaps(col);
		return new SphereShape(radius);
	}

	public function makeObject( ?material, ?parent ) : h3d.scene.Object {
		if ( prim == null ) {
			prim = new h3d.prim.Sphere(radius);
			prim.addNormals();
		}
		return new h3d.scene.Mesh(prim, material, parent);
	}
}

class Triangle extends Rigidbody {
	var mesh : h3d.scene.Object;
	var v0 : Vec3;
	var v1 : Vec3;
	var v2 : Vec3;
	var prim : h3d.prim.Polygon;

	public function new(v0 : Vec3, v1 : Vec3, v2 : Vec3, ?material, ?parent ) {
		super(parent);
		this.v0 = v0;
		this.v1 = v1;
		this.v2 = v2;
		mesh = makeObject(material, this);
	}

	public function makeShape() {
		return new TriangleShape(v0, v1, v2);
	}

	public function makeObject( ?material, ?parent ) : h3d.scene.Object {
		if ( prim == null ) {
			var idx = new hxd.IndexBuffer();
			idx.push(0);
			idx.push(1);
			idx.push(2);
			var points = [v0.toHeaps(), v1.toHeaps(), v2.toHeaps()];
			prim = new h3d.prim.Polygon(points, idx);
			prim.addNormals();
		}
		var mesh = new h3d.scene.Mesh(prim, material, parent);
		mesh.material.mainPass.culling = None;
		return mesh;
	}
}

class Compound extends Rigidbody {
	var sub : Array<{ body : Rigidbody, position : Vec3, rotation : Vec3 }>;
	var mesh : h3d.scene.Object;

	public function new(sub : Array<{ body : Rigidbody, position : Vec3, rotation : Vec3 }>, ?material, ?parent ) {
		super(parent);
		this.sub = sub;
		mesh = makeObject(material, this);
	}

	public function makeShape() {
		var c = new CompoundShape();
		for( s in sub ) {
			c.addSubShape(s.body.makeShape(), s.position, s.rotation);
		}
		return c;
	}

	public function makeObject( ?material, ?parent ) : h3d.scene.Object {
		var mesh = new h3d.scene.Object(parent);
		for( s in sub ) {
			var childObj = s.body.makeObject(material);
			childObj.setPosition(s.position.x, s.position.y, s.position.z);
			childObj.setRotation(s.rotation.x, s.rotation.y, s.rotation.z);
			mesh.addChild(childObj);
		}
		return mesh;
	}
}

class PhysicsSample extends hxd.App {

	var world : PhysicsWorld;
	var root : h3d.scene.Object;
	var profiler : MyProfiler;
	var console : ui.Console;

	var character : Rigidbody;
	var obstacle : Rigidbody;

	static inline final nbRandomBodies = 200;
	static inline final spawnRange = 10;
	static inline final speed = 5;
	static inline final debugContactPoint = false;
	static inline final forceWireframe = false;

	public function new() {
		h3d.mat.MaterialSetup.current = new h3d.mat.PbrMaterialSetup();
		super();
	}

	function createScene() {
		static var seed = 0;
		root = new h3d.scene.Object(s3d);
		var rand = new hxd.Rand(seed++);
		inline function randomOffset() { return ( rand.rand() - 0.5 ) * spawnRange; }
		inline function randomRotation() { return ( rand.rand() * hxd.Math.PI * 2.0 ) * spawnRange; }
		for ( _ in 0...nbRandomBodies ) {
			var m = h3d.mat.Material.create();
			m.color = new h3d.Vector4( rand.rand(), rand.rand(), rand.rand(), 1.0);
			if( forceWireframe || debugContactPoint )
				m.mainPass.wireframe = true;
			var type = Dynamic;
			var s : Rigidbody = switch( rand.random(9) ) {
				case 0:
					var r = 0.5 + (rand.rand() * 0.5);
					new Sphere(r, m, root);
				case 1:
					var r = 0.5 + (rand.rand() * 0.5);
					var h = 1 + (rand.rand() * 1);
					new Capsule(r, h, m, root);
				case 2:
					var sx = 0.75 + (rand.rand() * 0.5);
					var sy = 0.75 + (rand.rand() * 0.5);
					var sz = 0.75 + (rand.rand() * 0.5);
					new Box(new h3d.Vector(sx, sy, sz), m, root);
				case 3:
					// new HullSimple(rand.rand() + 0.5, rand.rand() + 1.0, m, root);
					new Hull("Tree01.FBX", root);
				case 4:
					var r = 0.5 + (rand.rand() * 0.5);
					var h = 1 + (rand.rand() * 1);
					new Cylinder(r, h, m, root);
				case 5:
					var v0 = new Vec3(rand.rand(), rand.rand(), rand.rand());
					var v1 = new Vec3(rand.rand(), rand.rand(), rand.rand());
					var v2 = new Vec3(rand.rand(), rand.rand(), rand.rand());
					new Triangle(v0, v1, v2, m, root);
				case 6:
					type = Static;
					new MeshSimple(m, root);
					// new Mesh("Tree01.FBX", m, root);
				case 7:
					var r = 0.5 + (rand.rand() * 0.5);
					var h = 1 + (rand.rand() * 1);
					var sx = 0.75 + (rand.rand() * 0.5);
					new Compound([
						{ body : new Cylinder(r, h), position : Vec3.zero(), rotation : Vec3.zero() },
						{ body : new Sphere(r), position : new Vec3(0.0, 0.0, h), rotation : Vec3.zero() },
						{ body : new Box(new h3d.Vector(sx, sx, sx)), position : new Vec3(0.0, r+sx, 0.0), rotation : Vec3.zero() },
					], m, root);
				case 8:
					type = Static;
					new Model("Tree01.FBX", m, root);
				default:
					throw "assert";
			};
			s.x = randomOffset();
			s.y = randomOffset();
			s.z = randomOffset();
			s.rotate(randomRotation(), randomRotation(), randomRotation());
			s.setScale(0.5 + rand.rand());
			s.createBody(world, type);
		}
	}

	function handleDebug() {
		static var g = null;
		static var debug = false;
		if (hxd.Key.isPressed(hxd.Key.F3))
			debug = !debug;
		if ( g != null )
			g.remove();
		if ( debug ) {
			g = @:privateAccess world.tree.makeDebugObj();
			s3d.addChild(g);
		}
	}

	override function init() {
		console = new ui.Console(hxd.res.DefaultFont.get(), s2d);
		profiler = new MyProfiler(s2d);
		world = new PhysicsWorld(nbRandomBodies+2, profiler);
		new h3d.scene.CameraController.OrbitCameraController(spawnRange * 5, s3d);
		createScene();
		var m = h3d.mat.Material.create();
		if( forceWireframe || debugContactPoint )
			m.mainPass.wireframe = true;
		// character = new Box(new h3d.Vector(1.0, 1.0, 1.0), m, s3d);
		character = new Capsule(0.5, 1.0, m, s3d);
		// character = new Cylinder(0.5, 1.0, m, s3d);
		// character = new HullSimple(0.5, 1.0, m, s3d);
		// character = new Hull("Tree01.FBX", m, s3d);
		// character = new Sphere(0.5, m, s3d);
		// character = new Compound([
		// 	{ body : new Cylinder(0.5, 1.0), position : Vec3.zero(), rotation : Vec3.zero() },
		// 	{ body : new Sphere(0.5), position : new Vec3(0.0, 0.0, 1), rotation : Vec3.zero() },
		// 	{ body : new Box(new h3d.Vector(0.5, 0.5, 0.5)), position : new Vec3(0.0, 1.0, 0.0), rotation : Vec3.zero() },
		// ], m, s3d);
		character.setPosition(0.0, 0.0, 10.0);
		character.createBody(world, Dynamic, CHARACTER_GROUP);
		obstacle = new Box(new h3d.Vector(100, 100, 1), s3d);
		obstacle.setPosition(0.0, 0.0, -15.0);
		obstacle.createBody(world, Static, OBSTACLE_GROUP);
	}

	function updateCharacter( dt : Float ) {
		var pt = new h3d.Vector();
		var angle : Float = 0.0;
		if( hxd.Key.isDown(hxd.Key.Z) )
			pt.x += dt * speed;
		if( hxd.Key.isDown(hxd.Key.S) )
			pt.x -= dt * speed;
		if( hxd.Key.isDown(hxd.Key.Q) )
			pt.y -= dt * speed;
		if( hxd.Key.isDown(hxd.Key.D) )
			pt.y += dt * speed;
		if( hxd.Key.isDown(hxd.Key.SPACE) )
			pt.z += dt * speed;
		if( hxd.Key.isDown(hxd.Key.CTRL) )
			pt.z -= dt * speed;
		if( hxd.Key.isDown(hxd.Key.SHIFT) )
			pt.scale(2);
		if( hxd.Key.isDown(hxd.Key.A) )
			angle -= hxd.Math.PI * dt;
		if( hxd.Key.isDown(hxd.Key.E) )
			angle += hxd.Math.PI * dt;
		var v = s3d.camera.target.sub(s3d.camera.pos);
		var a = hxd.Math.atan2(v.y, v.x);
		var c = Math.cos(a);
		var s = Math.sin(a);
		var px = pt.x * c - pt.y * s;
		var py = pt.x * s + pt.y * c;
		pt.x = px;
		pt.y = py;
		character.setPos(character.getAbsPos().getPosition().add(pt));
		var quat = new h3d.Quat();
		quat.initRotateAxis(0.0, 1.0, 0.0, angle);
		var qRot = character.getRotationQuat();
		quat.multiply(qRot, quat);
		character.setRot(quat);
	}

	override function update(dt : Float) {
		static var go = false;
		if (hxd.Key.isPressed(hxd.Key.F1))
			go = true;
		if (hxd.Key.isPressed(hxd.Key.F2)) {
			go = false;
			root.remove();
			createScene();
		}
		if ( hxd.Key.isPressed(hxd.Key.F5))
			world.enableThreading = !world.enableThreading;
		profiler.update();
		updateCharacter(dt);
		handleDebug();
		if ( !go )
			return;
		world.update();
		profiler.mark("Solving");

		if( !debugContactPoint ) {
			world.findCollisionsByPair(function(arr, start, count, b1Id, b2Id) {
				var c = arr.get(start);
				var b1 = world.getBody(b1Id);
				var b2 = world.getBody(b2Id);
				if ( b2.motionType != Dynamic )
					b1.position -= c.normal * c.penetration;
				else if ( b1.motionType != Dynamic )
					b2.position += c.normal * c.penetration;
				else {
					b1.position -= c.normal * c.penetration * 0.5;
					b2.position += c.normal * c.penetration * 0.5;
				}
			});
		}

		// Display some results
		if( !debugContactPoint ) {
			raycastFromPointer();
			shapecastFromCharacter();
		}
		if( debugContactPoint )
			collideFromCharacter();

		profiler.mark("Render");
	}

	function raycastFromPointer() {
		var rays3d = s3d.camera.rayFromScreen(s2d.mouseX, s2d.mouseY);
		var ray = Ray.fromHeaps(rays3d);
		var rayClosestHit = new HitResult();
		rayClosestHit.fraction = Math.SCALAR_MAX;
		var rayClosestBody = 0;

		world.raycast(ray, (hit, bodyID) -> {
			if ( hit.fraction < rayClosestHit.fraction ) {
				rayClosestHit.load(hit);
				rayClosestBody = bodyID;
			}
			return true;
		});

		static var gRayPointer : h3d.scene.Graphics = null;
		if ( gRayPointer == null ) {
			gRayPointer = new h3d.scene.Sphere(0.25, s3d);
			gRayPointer.material.mainPass.setPassName("overlay");
		}
		if ( rayClosestHit.fraction < Math.SCALAR_MAX ) {
			var p = ray.getPoint(rayClosestHit.fraction);
			gRayPointer.setPosition(p.x, p.y, p.z);
			gRayPointer.visible = true;
		} else {
			gRayPointer.visible = false;
		}
	}

	function shapecastFromCharacter() {
		var cBody = world.getBody(character.bodyID);
		static var gContacts : Array<{p:h3d.scene.Sphere, n:h3d.scene.Graphics}> = [];
		for( gc in gContacts ) {
			gc.p.visible = false;
			gc.n.visible = false;
			gc.n.clear();
		}
		var count = 0;

		var shapeCast = new ShapeCast();
		shapeCast.shape = cBody.shape;
		shapeCast.transform.initRotationQuat(cBody.rotation);
		shapeCast.transform.translate(cBody.position.x, cBody.position.y, cBody.position.z);

		static var directions = [
			new Vec3( 0.0, 0.0, -25.0 ), new Vec3( 0.0, 0.0, 25.0 ),
			new Vec3( 25.0, 0.0, 0.0 ), new Vec3( 0.0, 25.0, 0.0 ),
			new Vec3( -25.0, 0.0, 0.0 ), new Vec3( 0.0, -25.0, 0.0 ),
			// new Vec3( 0.0, 0.0, 0.0 ),
		];
		static var curDirIndex = 0;
		if( hxd.Key.isPressed(hxd.Key.N) ) {
			curDirIndex++;
			if( curDirIndex >= directions.length )
				curDirIndex = 0;
		}
		shapeCast.direction.load(directions[curDirIndex]);

		var closestHit = new ShapeCastResult();
		closestHit.fraction = Math.SCALAR_MAX;
		world.shapecast(shapeCast, (hit, bodyID) -> {
			if ( hit.fraction < closestHit.fraction ) {
				closestHit.load(hit);
			}
			var gc = gContacts[count];
			if( gc == null ) {
				var p = new h3d.scene.Sphere(0xFFFF0000, 0.25, s3d);
				var n = new h3d.scene.Graphics(s3d);
				n.lineStyle(2.0, 0x15FF00);
				n.material.mainPass.setPassName("overlay");
				gc = { p : p, n : n };
				gContacts[count] = gc;
			}
			gc.p.visible = true;
			gc.n.visible = true;
			var normal = hit.getNormal();
			var p = hit.contactPointOn2;
			gc.p.setPosition(p.x, p.y, p.z);
			gc.n.moveTo(p.x, p.y, p.z);
			gc.n.lineTo(p.x + normal.x, p.y + normal.y, p.z + normal.z);
			count++;
			return true;
		}, 1.0, DEFAULT_GROUP, ClosestPerBody);

		static var gRay : h3d.scene.Graphics = null;
		if ( gRay == null ) {
			gRay = new h3d.scene.Graphics(s3d);
			gRay.material.mainPass.setPassName("overlay");
			gRay.lineStyle(1.0, 0x0000FF, 0.5);
		}

		static var gPreviz : h3d.scene.Object = null;
		if( gPreviz != null )
			gPreviz.remove();

		gRay.clear();
		var origin = shapeCast.transform.getPosition();
		gRay.moveTo(origin.x, origin.y, origin.z);
		if ( closestHit.fraction < Math.SCALAR_MAX ) {
			var material = h3d.mat.Material.create();
			material.blendMode = Alpha;
			material.color = new h3d.Vector4(1.0, 1.0, 1.0, 0.75);
			gPreviz = character.makeObject(material, s3d);
			gPreviz.setRotationQuat(character.getRotationQuat());

			var normal = closestHit.getNormal();
			var p = shapeCast.getPoint(closestHit.fraction) + normal * closestHit.penetration;
			gPreviz.setPosition(p.x, p.y, p.z);
			gRay.lineTo(p.x, p.y, p.z);
		} else {
			gRay.lineTo(origin.x + shapeCast.direction.x, origin.y + shapeCast.direction.y, origin.z + shapeCast.direction.z);
		}
	}

	function collideFromCharacter() {
		var cBody = world.getBody(character.bodyID);
		static var gContacts : Array<{p1:h3d.scene.Sphere, p2:h3d.scene.Sphere, n:h3d.scene.Graphics}> = [];
		for( gc in gContacts ) {
			gc.p1.visible = false;
			gc.p2.visible = false;
			gc.n.visible = false;
			gc.n.clear();
		}
		var count = 0;
		world.collide(cBody.shape, cBody.scale, cBody.transform, function(contact, bodyID) {
			if( bodyID == character.bodyID )
				return true;
			var gc = gContacts[count];
			if( gc == null ) {
				var p1 = new h3d.scene.Sphere(0xFFFF0000, 0.2, s3d);
				var p2 = new h3d.scene.Sphere(0xFF00F7FF, 0.3, s3d);
				var n = new h3d.scene.Graphics(s3d);
				n.lineStyle(2.0, 0x15FF00);
				n.material.mainPass.setPassName("overlay");
				gc = { p1 : p1, p2 : p2, n : n };
				gContacts[count] = gc;
			}
			gc.p1.visible = true;
			gc.p2.visible = true;
			gc.n.visible = true;
			var p1 = contact.contactPointOn1;
			gc.p1.setPosition(p1.x, p1.y, p1.z);
			var p2 = contact.contactPointOn2;
			gc.p2.setPosition(p2.x, p2.y, p2.z);
			var normal = contact.normal * contact.penetration;
			gc.n.moveTo(p2.x, p2.y, p2.z);
			gc.n.lineTo(p2.x + normal.x, p2.y + normal.y, p2.z + normal.z);
			count++;
			return true;
		});
	}

	static function main() {
		hxd.Res.initLocal();
		new PhysicsSample();
	}
}
