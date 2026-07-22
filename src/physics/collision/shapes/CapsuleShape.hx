package physics.collision.shapes;

class CapsuleSupport extends ConvexSupport {
	public var radius : Scalar;
	public var halfHeight : Scalar;

	public inline function init( shape : Shape, scale : Vec3 ) {
		var capsule : CapsuleShape = hl.Api.unsafeCast(shape);
		this.radius = capsule.radius * scale.x;
		this.halfHeight = capsule.halfHeight * scale.x;
	}

	public inline function getSupportWithMargin( dir : Vec3, out : Vec3 ) {
		getSupport(dir, out);
		var len = dir.length();
		if( len > 0.0 )
			out.add(dir * ( radius / len ));
	}

	public inline function getSupport( dir : Vec3, out : Vec3 ) {
		out.set(0.0, 0.0, dir.z > 0 ? halfHeight : -halfHeight);
	}

	public inline function getMargin() : Scalar {
		return radius;
	}
}

class CapsuleShape extends ConvexShape {
	public var radius : Scalar;
	public var halfHeight : Scalar;

	public inline function new( radius : Scalar, halfHeight : Scalar ) {
		shapeType = Capsule;
		this.radius = radius;
		this.halfHeight = halfHeight;
	}

	public function toString() {
		return "Capsule";
	}

	public inline function getLocalBounds() {
		var halfHeightWithRadius = halfHeight + radius;
		return new AABB(new Vec3(-radius, -radius, -halfHeightWithRadius), new Vec3(radius, radius, halfHeightWithRadius));
	}

	public inline function getLocalBoundsToBuffer( out : AABB ) {
		out.load(getLocalBounds());
	}

	public inline function getSurfaceNormal( localPos : Vec3 ) : Vec3 {
		var normal = new Vec3();
		if ( localPos.z > halfHeight )
			normal.load((localPos - new Vec3(0.0, 0.0, halfHeight)).normalized());
		else if ( localPos.z < -halfHeight)
			normal.load((localPos - new Vec3(0.0, 0.0, -halfHeight)).normalized());
		else
			normal.load(new Vec3(localPos.x, localPos.y, 0.0).normalized());
		return normal;
	}

	public inline function raycast( ray : Ray, scale : Vec3, transform : Mat, infos : HitResult ) : Bool {
		var invTransform = transform.getInverse();
		var torigin = ray.origin.transformed(invTransform);
		var tdirection = ray.direction.transformed3x3(invTransform);
		var fraction = Math.localRayCapsule(torigin, tdirection, halfHeight * scale.x, radius * scale.x);
		if ( fraction < Math.SCALAR_MAX ) {
			infos.position.load(ray.getPoint(fraction));
			infos.normal.load(getSurfaceNormal(torigin + tdirection * fraction).transformed3x3(transform));
			infos.fraction = fraction;
			return true;
		}
		return false;
	}

	public inline function isScaleValid( scale : Vec3 ) {
		return !ScaleHelper.isNearZero(scale) && ScaleHelper.isUniform(scale);
	}

	public inline function makeScaleValid( scale : Vec3 ) {
		scale.load(ScaleHelper.makeNonZero(scale));
		scale.load(ScaleHelper.makeUniform(scale));
	}

	public inline function getSupportClass() {
		return CapsuleSupport;
	}

	#if heaps
	public static function fromHeaps( capsule : h3d.col.Capsule, position : Vec3, rotation : Vec3 ) : CapsuleShape {
		var dir = capsule.a - capsule.b;
		var halfHeight = dir.length() * (1/2);
		var shape = new CapsuleShape(capsule.r, halfHeight);
		var pos = (capsule.a + capsule.b) * (1/2);
		var rot = Quat.fromTo(new Vec3(0.0, 0.0, 1.0), Vec3.fromHeaps(dir.normalized())).getEulerAngles();
		position.set(pos.x, pos.y, pos.z);
		rotation.set(rot.x, rot.y, rot.z);
		return shape;
	}
	#end
}
