package physics.collision.shapes;

class SubShape {
	public var shape : Shape;
	@:packed public var position : Vec3;
	@:packed public var rotation : Quat;
	public var isRotationIdentity : Bool;

	@:packed var transform : Mat;

	public function new( shape : Shape, position : Vec3, rotation : Quat ) {
		this.shape = shape;
		this.position.load(position);
		if( rotation.isClose(Quat.identity()) || rotation.isClose(Quat.identityNeg()) ) {
			isRotationIdentity = true;
			this.rotation.load(Quat.identity());
		} else {
			isRotationIdentity = false;
			this.rotation.load(rotation);
		}
	}

	public inline function getTransformMatrix( pscale : Vec3 ) : Mat {
		Assert.t(isScaleValid(pscale));
		transform.initRotationQuat(rotation);
		transform.translate(position.x * pscale.x, position.y * pscale.y, position.z * pscale.z);
		return transform;
	}

	public inline function transformScale( pscale : Vec3 ) : Vec3 {
		var nscale = Vec3.one();
		if( isRotationIdentity || ScaleHelper.isUniform(pscale) )
			nscale.load(pscale);
		else
			nscale.load(ScaleHelper.rotateScale(rotation, pscale));
		return nscale;
	}

	public inline function isScaleValid( pscale : Vec3 ) {
		if( isRotationIdentity || ScaleHelper.isUniform(pscale) )
			return true;
		return ScaleHelper.canScaleBeRotated(rotation, pscale);
	}
}

class CompoundShape extends Shape {

	public var subShapes(default, null) : Array<SubShape>;
	var localBounds : AABB;

	@:packed var tmpVec : Vec3;

	public inline function new() {
		shapeType = Compound;
		subShapes = [];
		localBounds = AABB.empty();
	}

	public function toString() {
		return "Compound" + [for( s in subShapes ) s.shape.toString()];
	}

	override function mustBeStatic() : Bool {
		for( sub in subShapes ) {
			if( sub.shape.mustBeStatic() )
				return true;
		}
		return false;
	}

	override function buildLater() : Void {
		for( sub in subShapes )
			sub.shape.buildLater();
	}

	public inline function getLocalBounds() {
		return localBounds;
	}

	public inline function getLocalBoundsToBuffer( out : AABB ) {
		out.load(getLocalBounds());
	}

	public inline function getSurfaceNormal( localPos : Vec3 ) : Vec3 {
		throw "Not implemented";
	}

	public function raycast( ray : Ray, scale : Vec3, transform : Mat, infos : HitResult ) : Bool {
		throw "Should not be called directly"; // See CompoundAlgorithm.raycast
	}

	public function addSubShape( shape : Shape, position : Vec3 = null, rotation : Vec3 = null ) {
		if( position == null )
			position = Vec3.zero();
		if( rotation == null )
			rotation = Vec3.zero();
		var qrot = Quat.identity();
		qrot.initRotation(rotation.x, rotation.y, rotation.z);
		var sub = new SubShape(shape, position, qrot);
		subShapes.push(sub);
		var m = sub.getTransformMatrix(Vec3.one());
		var subBounds = shape.getLocalBounds().transformed(m);
		localBounds.merge(subBounds);
	}

	public inline function isScaleValid( scale : Vec3 ) {
		if( ScaleHelper.isNearZero(scale) )
			return false;
		if( ScaleHelper.isNotScaled(scale) )
			return true;
		var valid = true;
		for( sub in subShapes ) {
			if( !sub.isScaleValid(scale) ) {
				valid = false;
				break;
			}
			tmpVec.load(sub.transformScale(scale));
			if( !sub.shape.isScaleValid(tmpVec) ) {
				valid = false;
				break;
			}
		}
		return valid;
	}

	public inline function makeScaleValid( scale : Vec3 ) {
		scale.load(ScaleHelper.makeNonZero(scale));
		if( isScaleValid(scale) )
			return;
		scale.load(ScaleHelper.makeUniform(scale));
	}
}
