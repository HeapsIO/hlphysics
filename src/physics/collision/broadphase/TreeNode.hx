package physics.collision.broadphase;

@:struct class TreeNode {
	public static inline final NULL_TREE_NODE : Int = -1;

	public var nextID : Int = NULL_TREE_NODE;
	public var leftChild : Int;
	public var rightChild : Int;

	public var bodyID : BodyID;

	public var height : Int = -1;

	@:packed public var aabb : AABB;

	public function new() {}

	public function getParentID() : Int { return nextID; }
	public function getNextNodeID() : Int { return nextID; }

	public function isLeaf() : Bool { return height == 0; }
}
