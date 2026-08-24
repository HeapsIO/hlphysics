package physics.collision.shapes;

enum abstract ShapeType(Int) from Int to Int {
	var Sphere = 0;
	var Box;
	var Capsule;
	var Cylinder;
	var ConvexHull;
	var Triangle;
	// --- Not always convex
	var Mesh;
	var Compound;
	var Scaled;
	var Empty;
	// --- Last
	var ShapeCount;

	public function isConvex() {
		return this < (Mesh:Int);
	}

	public function toString() {
		return switch (this) {
		case Sphere: "Sphere";
		case Box: "Box";
		case Capsule: "Capsule";
		case Cylinder: "Cylinder";
		case ConvexHull: "ConvexHull";
		case Triangle: "Triangle";
		case Mesh: "Mesh";
		case Compound: "Compound";
		case Scaled: "Scaled";
		case Empty: "Empty";
		default: "Unknown";
		}
	}
}

abstract class Shape {
	var shapeType : ShapeType;
	#if physics_profile
	public var profileTag : String;
	#end

	public inline function getType() { return shapeType; }
	public inline function isConvex() : Bool { return shapeType.isConvex(); }
	public abstract function toString() : String;
	public function mustBeStatic() : Bool { return false; }
	public abstract function getLocalBounds() : AABB;
	/**
		Similar to `getLocalBounds()` but use `out` buffer to reduce alloc.
	**/
	public abstract function getLocalBoundsToBuffer( out : AABB ) : Void;
	public abstract function getSurfaceNormal( localPoint : Vec3 ) : Vec3;
	public abstract function raycast( ray : Ray, scale : Vec3, transform : Mat, infos : HitResult ) : Bool;
	public abstract function isScaleValid( scale : Vec3 ) : Bool;
	public abstract function makeScaleValid( scale : Vec3 ) : Void;

	public static inline function transformed( shape : Shape, position : Vec3, rotation : Vec3, scale : Vec3 ) : Shape {
		if( ScaleHelper.isNearZero(scale) )
			throw "Invalid scale " + scale;
		if( !ScaleHelper.isNotScaled(scale) )
			shape = new ScaledShape(shape, scale);
		if( position.equals(Vec3.zero()) && rotation.equals(Vec3.zero()) )
			return shape;
		var compound = new CompoundShape();
		compound.addSubShape(shape, position.clone(), rotation.clone());
		return compound;
	}

	#if heaps
	/**
		Convert heaps collider to Shape.
		If `follows` is given empty, the first `ObjectCollider` found is recorded into it as the root; pass it
		pre-filled with a single object to force that object as the root instead. Any other `ObjectCollider`
		found is baked relative to the root.
	**/
	public static function fromHeaps( col : h3d.col.Collider, ?follows : Array<h3d.scene.Object> ) : Shape {
		if( col == null )
			return new EmptyShape();
		var obj = Std.downcast(col, h3d.col.ObjectCollider);
		if( obj != null ) {
			var relative : h3d.Matrix = null;
			if( follows != null ) {
				if( follows.length == 0 )
					follows.push(obj.obj);
				else if( follows[0] != obj.obj ) {
					relative = obj.obj.getAbsPos().clone();
					relative.multiply(relative, follows[0].getInvPos());
				}
			}
			var s = Shape.fromHeaps(obj.collider, follows);
			if( relative != null )
				s = Shape.transformed(s, Vec3.fromHeaps(relative.getPosition()), Vec3.fromHeaps(relative.getEulerAngles()), Vec3.fromHeaps(relative.getScale()));
			return s;
		}
		var opt = Std.downcast(col, h3d.col.Collider.OptimizedCollider);
		if( opt != null ) {
			return Shape.fromHeaps(opt.b, follows);
		}
		var trans = Std.downcast(col, h3d.col.TransformCollider);
		if( trans != null ) {
			var mat = trans.mat;
			var s = Shape.fromHeaps(trans.collider, follows);
			return Shape.transformed(s, Vec3.fromHeaps(mat.getPosition()), Vec3.fromHeaps(mat.getEulerAngles()), Vec3.fromHeaps(mat.getScale()));
		}
		var position = new Vec3();
		var rotation = new Vec3();
		var group = Std.downcast(col, h3d.col.Collider.GroupCollider);
		if( group != null ) {
			var compound = new CompoundShape();
			for( c in group.colliders ) {
				var s = Shape.fromHeaps(c, follows);
				var sc = Std.downcast(s, CompoundShape);
				if( sc != null ) {
					for( ss in sc.subShapes ) {
						compound.addSubShape(ss.shape, ss.position, ss.rotation.getEulerAngles());
					}
				} else {
					compound.addSubShape(s);
				}
			}
			return compound;
		}
		var shape = Shape.fromHeapsSimple(col, position, rotation);
		if( shape != null ) {
			return Shape.transformed(shape, position, rotation, Vec3.one());
		}
		throw "Don't know how to convert shape " + col;
	}

	static function fromHeapsSimple( col : h3d.col.Collider, position : Vec3, rotation : Vec3 ) : Shape {
		var sphere = Std.downcast(col, h3d.col.Sphere);
		if( sphere != null )
			return SphereShape.fromHeaps(sphere, position, rotation);
		var box = Std.downcast(col, h3d.col.OrientedBounds);
		if( box != null )
			return BoxShape.fromHeaps(box, position, rotation);
		var bounds = Std.downcast(col, h3d.col.Bounds);
		if( bounds != null )
			return BoxShape.fromHeapsBounds(bounds, position, rotation);
		var capsule = Std.downcast(col, h3d.col.Capsule);
		if( capsule != null )
			return CapsuleShape.fromHeaps(capsule, position, rotation);
		var cylinder = Std.downcast(col, h3d.col.Cylinder);
		if( cylinder != null )
			return CylinderShape.fromHeaps(cylinder, position, rotation);
		var polygon = Std.downcast(col, h3d.col.Polygon);
		if( polygon != null ) {
			position.set(0.0, 0.0, 0.0);
			rotation.set(0.0, 0.0, 0.0);
			var vertices : Array<Single> = [];
			var indexes : Array<Int> = [];
			for( idx => p in polygon.getPoints() ) {
				vertices.push(p.x);
				vertices.push(p.y);
				vertices.push(p.z);
				indexes.push(idx);
			}
			if( polygon.isConvex() )
				return new ConvexHullShape(vertices, indexes);
			else
				return new MeshShape(vertices, indexes);
		}
		var polygonBuffer = Std.downcast(col, h3d.col.PolygonBuffer);
		if( polygonBuffer != null ) {
			position.set(0.0, 0.0, 0.0);
			rotation.set(0.0, 0.0, 0.0);
			if( polygonBuffer.isConvex )
				return ConvexHullShape.fromHeaps(polygonBuffer);
			else
				return MeshShape.fromHeaps(polygonBuffer);
		}
		var skin = Std.downcast(col, h3d.col.SkinCollider);
		if( skin != null ) {
			@:privateAccess skin.applyTransform();
			var poly = @:privateAccess skin.transform;
			position.set(0.0, 0.0, 0.0);
			rotation.set(0.0, 0.0, 0.0);
			return MeshShape.fromHeaps(poly);
		}
		return null;
	}
	#end
}
