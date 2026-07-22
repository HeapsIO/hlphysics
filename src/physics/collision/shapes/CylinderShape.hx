package shiro.physics.collision.shapes;

class CylinderSupport extends ConvexSupport {
	public var radius : Scalar;
	public var halfHeight : Scalar;

	public inline function init( shape : Shape, scale : Vec3 ) {
		var cylinder : CylinderShape = hl.Api.unsafeCast(shape);
		this.radius = cylinder.radius * scale.x;
		this.halfHeight = cylinder.halfHeight * scale.z;
	}

	public inline function getSupportWithMargin( dir : Vec3, out : Vec3 ) {
		var x = dir.x;
		var y = dir.y;
		var z = dir.z;
		var o = Math.sqrt(x*x + y*y);
		if ( o > 0.0 )
			out.set(radius * x / o, radius * y / o, (z >= 0 ? 1.0 : -1.0) * halfHeight);
		else
			out.set(0.0, 0.0, (z >= 0 ? 1.0 : -1.0) * halfHeight);
	}

	public inline function getSupport( dir : Vec3, out : Vec3 ) {
		var x = dir.x;
		var y = dir.y;
		var z = dir.z;
		var halfHeight = halfHeight - getMargin();
		var radius = radius - getMargin();
		var o = Math.sqrt(x*x + y*y);
		if ( o > 0.0 )
			out.set(radius * x / o, radius * y / o, (z >= 0 ? 1.0 : -1.0) * halfHeight);
		else
			out.set(0.0, 0.0, (z >= 0 ? 1.0 : -1.0) * halfHeight);
	}

	public inline function getMargin() : Scalar {
		return 0.05;
	}
}

class CylinderShape extends ConvexShape {

	public var radius : Scalar;
	public var halfHeight : Scalar;

	public inline function new( radius : Scalar, halfHeight : Scalar ) {
		shapeType = Cylinder;
		this.radius = radius;
		this.halfHeight = halfHeight;
	}

	public function toString() : String {
		return "Cylinder";
	}

	public inline function getLocalBounds() {
		return new AABB(new Vec3(-radius, -radius, -halfHeight), new Vec3(radius, radius, halfHeight));
	}

	public inline function getLocalBoundsToBuffer( out : AABB ) {
		out.load(getLocalBounds());
	}

	public inline function getSurfaceNormal( localPos : Vec3 ) : Vec3 {
		var localPosXY = new Vec3(localPos.x, localPos.y, 0.0);
		var localPosXYLen = localPosXY.length();
		var distToCurvedSurface = Math.abs(localPosXYLen - radius);

		var distToTopOrBottom = Math.abs(Math.abs(localPos.z) - halfHeight);

		var normal = new Vec3();
		if (distToCurvedSurface < distToTopOrBottom)
			normal.load(localPosXY.scaled(1.0 / localPosXYLen));
		else
			normal.set(0.0, 0.0, Math.sign(localPos.z));
		return normal;
	}

	public inline function raycast( ray : Ray, scale : Vec3, transform : Mat, infos : HitResult ) : Bool {
		var invTransform = transform.getInverse();
		var torigin = ray.origin.transformed(invTransform);
		var tdirection = ray.direction.transformed3x3(invTransform);
		var fraction = Math.localRayCylinder(torigin, tdirection, halfHeight * scale.z, radius * scale.x);
		if ( fraction < Math.SCALAR_MAX ) {
			infos.position.load(ray.getPoint(fraction));
			infos.normal.load(getSurfaceNormal(torigin + tdirection * fraction).transformed3x3(transform));
			infos.fraction = fraction;
			return true;
		}
		return false;
	}

	public inline function isScaleValid( scale : Vec3 ) {
		return !ScaleHelper.isNearZero(scale) && ScaleHelper.isUniformXY(scale);
	}

	public inline function makeScaleValid( scale : Vec3 ) {
		scale.load(ScaleHelper.makeNonZero(scale));
		scale.load(ScaleHelper.makeUniformXY(scale));
	}

	public inline function getSupportClass() {
		return CylinderSupport;
	}

	#if heaps
	public static function fromHeaps( cylinder : h3d.col.Cylinder, position : Vec3, rotation : Vec3 ) : CylinderShape {
		var dir = cylinder.a - cylinder.b;
		var halfHeight = dir.length() * (1/2);
		var shape = new CylinderShape(cylinder.r, halfHeight);
		var pos = (cylinder.a + cylinder.b) * (1/2);
		var rot = Quat.fromTo(new Vec3(0.0, 0.0, 1.0), Vec3.fromHeaps(dir.normalized())).getEulerAngles();
		position.set(pos.x, pos.y, pos.z);
		rotation.set(rot.x, rot.y, rot.z);
		return shape;
	}
	#end
}
