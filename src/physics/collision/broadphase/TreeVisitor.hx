package physics.collision.broadphase;

abstract class TreeVisitor {
	public var stack : hl.NativeArray<Int>;
	public var inUse : Bool;
	public abstract function visitNode( node : TreeNode ) : Bool;
	public abstract function visitBody( body : TreeNode ) : Bool;
}
