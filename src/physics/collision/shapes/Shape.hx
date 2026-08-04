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
	public static function fromHeaps( col : h3d.col.Collider ) : Shape {
		if( col == null )
			return new EmptyShape();
		var obj = Std.downcast(col, h3d.col.ObjectCollider);
		if( obj != null ) {
			return Shape.fromHeaps(obj.collider);
		}
		var opt = Std.downcast(col, h3d.col.Collider.OptimizedCollider);
		if( opt != null ) {
			return Shape.fromHeaps(opt.b);
		}
		var trans = Std.downcast(col, h3d.col.TransformCollider);
		if( trans != null ) {
			var mat = trans.mat;
			var s = Shape.fromHeaps(trans.collider);
			return Shape.transformed(s, Vec3.fromHeaps(mat.getPosition()), Vec3.fromHeaps(mat.getEulerAngles()), Vec3.fromHeaps(mat.getScale()));
		}
		var position = new Vec3();
		var rotation = new Vec3();
		var group = Std.downcast(col, h3d.col.Collider.GroupCollider);
		if( group != null ) {
			var compound = new CompoundShape();
			for( c in group.colliders ) {
				var s = Shape.fromHeaps(c);
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
		var capsule = Std.downcast(col, h3d.col.Capsule);
		if( capsule != null )
			return CapsuleShape.fromHeaps(capsule, position, rotation);
		var cylinder = Std.downcast(col, h3d.col.Cylinder);
		if( cylinder != null )
			return CylinderShape.fromHeaps(cylinder, position, rotation);
		var poly = Std.downcast(col, h3d.col.PolygonBuffer);
		if( poly != null ) {
			position.set(0.0, 0.0, 0.0);
			rotation.set(0.0, 0.0, 0.0);
			if( poly.isConvex )
				return ConvexHullShape.fromHeaps(poly);
			else
				return MeshShape.fromHeaps(poly);
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
