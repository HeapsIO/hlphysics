package physics.collision.shapes;

/**
	Note : new() should do nothing, because instances are created using `Type.createEmptyInstance`.
**/
abstract class ConvexSupport {
	public final inline function new() {}
	public abstract function init( shape : Shape, scale : Vec3 ) : Void;
	public abstract function getSupportWithMargin( dir : Vec3, out : Vec3 ) : Void;
	public abstract function getSupport( dir : Vec3, out : Vec3 ) : Void;
	public abstract function getMargin() : Scalar;
}

abstract class ConvexShape extends Shape {
	public abstract function getSupportClass() : Class<ConvexSupport>;

	/**
		GetSupport function need to be call with the support of the specific class
	**/
	public function getSupportFunction( support : ConvexSupport, scale : Vec3 ) : Void {
		Assert.t(isScaleValid(scale));
		support.init(this, scale);
	}
}
