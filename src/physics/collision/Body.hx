package physics.collision;

enum MotionType {
	Static;
	Kinematic;
	Dynamic;
}

class Body {
	@:packed public var position : Vec3;
	@:packed public var rotation : Quat;
	/**
		Computed based on `position`, `rotation`.
	**/
	@:packed public var transform(get, null) : Mat;
	@:packed public var scale(default, set) : Vec3;

	public var nodeID : Int;
	public var motionType(default, null) : MotionType;
	public var collisionGroup : Int;
	public var collisionMask : Int;
	public var shape : Shape;
	public var userData : Dynamic;

	/**
		Should be set manually if change position/rotation directly
	**/
	public var transformChanged : Bool;

	public function new( shape : Shape ) {
		this.shape = shape;
		position.load(Vec3.zero());
		rotation.load(Quat.identity());
		scale.load(Vec3.one());
		transform.load(Mat.identity());
		motionType = Static;
		collisionGroup = 1;
		collisionMask = ~0;
		transformChanged = false;
	}

	public function load( b : Body ) {
		shape = b.shape;
		position.load(b.position);
		rotation.load(b.rotation);
		scale.load(b.scale);
		nodeID = b.nodeID;
		motionType = b.motionType;
		collisionGroup = b.collisionGroup;
		collisionMask = b.collisionMask;
		userData = b.userData;
		transformChanged = true;
	}

	public inline function getWorldBounds() {
		return shape.getLocalBounds().scaled(scale).transformed(transform);
	}

	public inline function setMotionType( motionType : MotionType ) {
		if( motionType != Static && shape.mustBeStatic() ) {
			throw "Can't set MotionType " + motionType + " for " + shape;
		}
		this.motionType = motionType;
	}

	public inline function setPosition( x : Scalar, y : Scalar, z : Scalar ) {
		position.x = x;
		position.y = y;
		position.z = z;
		transformChanged = true;
	}

	public inline function setRotation( ax : Scalar, ay : Scalar, az : Scalar ) {
		rotation.initRotation(ax, ay, az);
		transformChanged = true;
	}

	public inline function setScale( sx : Scalar, sy : Scalar, sz : Scalar ) {
		scale.x = sx;
		scale.y = sy;
		scale.z = sz;
		Assert.w(shape.isScaleValid(scale), "Invalid scale " + scale + " for " + shape.toString());
	}

	inline function set_scale( s : Vec3 ) {
		scale.load(s);
		Assert.w(shape.isScaleValid(scale), "Invalid scale " + scale + " for " + shape.toString());
		return scale;
	}

	inline function get_transform() {
		if( transformChanged ) {
			transform.initRotationQuat(rotation);
			transform.translate(position.x, position.y, position.z);
			transformChanged = false;
		}
		return transform;
	}

	public inline function filter( mask : Int ) : Bool {
		return collisionGroup & mask != 0;
	}

	public function toString() {
		return '{position:$position,rotation:$rotation,nodeID:$nodeID,collisionGroup:$collisionGroup,collisionMask:$collisionMask,shape:$shape}';
	}
}
