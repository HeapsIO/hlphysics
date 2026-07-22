package physics.collision.shapes;

class BoxSupport extends ConvexSupport {
	@:packed public var halfExtent : Vec3;

	public inline function init( shape : Shape, scale : Vec3 ) {
		var box : BoxShape = hl.Api.unsafeCast(shape);
		this.halfExtent.load(box.halfExtent.multiplied(scale));
	}

	public inline function getSupportWithMargin( dir : Vec3, out : Vec3 ) {
		out.set(
			dir.x < 0 ? -halfExtent.x : halfExtent.x,
			dir.y < 0 ? -halfExtent.y : halfExtent.y,
			dir.z < 0 ? -halfExtent.z : halfExtent.z
		);
	}

	public inline function getSupport( dir : Vec3, out : Vec3 ) {
		var halfExtent = halfExtent.clone();
		var margin = getMargin();
		halfExtent.x -= margin;
		halfExtent.y -= margin;
		halfExtent.z -= margin;
		out.set(
			dir.x < 0 ? -halfExtent.x : halfExtent.x,
			dir.y < 0 ? -halfExtent.y : halfExtent.y,
			dir.z < 0 ? -halfExtent.z : halfExtent.z
		);
	}

	public inline function getMargin() : Scalar {
		return 0.005;
	}
}

class BoxShape extends ConvexPolyhedronShape {
	@:packed public var halfExtent : Vec3;

	public inline function new( halfExtent : Vec3 ) {
		shapeType = Box;
		this.halfExtent = halfExtent;
	}

	public function toString() {
		return "Box";
	}

	public inline function getLocalBounds() {
		return new AABB(halfExtent.scaled(-1), halfExtent.clone());
	}

	public inline function getLocalBoundsToBuffer( out : AABB ) {
		out.load(getLocalBounds());
	}

	public inline function getSurfaceNormal( localPos : Vec3 ) : Vec3 {
		var index = (localPos.abs() - halfExtent).abs().getLowestComponentIndex();
		var normal = new Vec3(0.0, 0.0, 0.0);
		normal[index] = localPos[index] > 0.0 ? 1.0 : -1.0;
		return normal;
	}

	public inline function raycast( ray : Ray, scale : Vec3, transform : Mat, infos : HitResult ) : Bool {
		var invTransform = transform.getInverse();
		var torigin = ray.origin.transformed(invTransform);
		var tdirection = ray.direction.transformed3x3(invTransform);
		var fraction = Math.localRayBox(torigin, tdirection, halfExtent.multiplied(scale));
		if ( fraction < Math.SCALAR_MAX ) {
			infos.position.load(ray.getPoint(fraction));
			infos.normal.load(getSurfaceNormal(torigin + tdirection * fraction).transformed3x3(transform));
			infos.fraction = fraction;
			return true;
		}
		return false;
	}

	public inline function isScaleValid( scale : Vec3 ) : Bool {
		return !ScaleHelper.isNearZero(scale);
	}

	public inline function makeScaleValid( scale : Vec3 ) {
		scale.load(ScaleHelper.makeNonZero(scale));
	}

	public inline function getSupportClass() {
		return BoxSupport;
	}

	#if heaps
	public static function fromHeaps( box : h3d.col.OrientedBounds, position : Vec3, rotation : Vec3 ) : BoxShape {
		var m = box.getMatrix();
		var halfExtent = m.getScale() * (1/2);
		var pos = m.getPosition();
		var rot = m.getEulerAngles();
		var shape = new BoxShape(Vec3.fromHeaps(halfExtent));
		position.set(pos.x, pos.y, pos.z);
		rotation.set(rot.x, rot.y, rot.z);
		return shape;
	}
	#end
}
