package physics.collision.broadphase;

#if heaps
private class DebugVisitor extends TreeVisitor {
	public var root : h3d.scene.Object;
	var rand : hxd.Rand;

	public inline function new() {
		root = new h3d.scene.Object();
		rand = new hxd.Rand(0);
	}

	public inline function init() {
		root.removeChildren();
	}

	inline function toBounds( aabb : AABB ) {
		var bounds = new h3d.col.Bounds();
		bounds.xMax = aabb.max.x;
		bounds.yMax = aabb.max.y;
		bounds.zMax = aabb.max.z;
		bounds.xMin = aabb.min.x;
		bounds.yMin = aabb.min.y;
		bounds.zMin = aabb.min.z;
		return bounds;
	}

	public inline function visitNode( node : TreeNode ) {
		visitBody(node);
		return true;
	}

	public inline function visitBody( body : TreeNode ) {
		rand.init(body.height);
		var color = new h3d.Vector(rand.rand(),rand.rand(),rand.rand());
		root.addChild(new h3d.scene.Box(color.toColor(), toBounds(body.aabb), true));
		return true;
	}
}
#end

class AABBTree {
	var nodes : StaticArray<TreeNode>;
	var rootID : Int;
	var freeID : Int;
	var allocatedNodeCount : Int;
	var nodeCount : Int;
	var aabbInflate : Scalar;

	public function new( aabbInflate : Scalar, initSize : Int = 16 ) {
		this.aabbInflate = aabbInflate;
		init(initSize);
	}

	function init( initSize : Int ) {
		rootID = TreeNode.NULL_TREE_NODE;
		nodeCount = 0;
		allocatedNodeCount = initSize;

		nodes = new StaticArray(TreeNode, allocatedNodeCount);

		// Initialize the allocated nodes
		for (i in 0...allocatedNodeCount) {
			var n = nodes.pushEmpty();
			n.nextID = i + 1;
			n.height = -1;
		}

		var lastNode = nodes.get(allocatedNodeCount - 1);
		lastNode.nextID = TreeNode.NULL_TREE_NODE;
		lastNode.height = -1;
		freeID = 0;
	}

	public function reset( initSize : Int = 16 ) {
		init(initSize);
	}

	public inline function addBody(aabb : AABB, id : BodyID ) : Int {
		var nodeID = addObjectInternal(aabb);

		var node = nodes.get(nodeID);
		node.bodyID = id;

		return nodeID;
	}

	function allocNode() : Int {
		if ( freeID == TreeNode.NULL_TREE_NODE ) {
			Assert.t( nodeCount == allocatedNodeCount );

			allocatedNodeCount *= 2;
			nodes.resize(allocatedNodeCount);

			for ( i in nodeCount...allocatedNodeCount ) {
				var n = nodes.pushEmpty();
				n.nextID = i + 1;
				n.height = -1;
			}
			var lastNode = nodes.get(allocatedNodeCount - 1);
			lastNode.nextID = TreeNode.NULL_TREE_NODE;
			lastNode.height = -1;
			freeID = nodeCount;
		}

		var id = freeID;
		var node = nodes.get(id);
		freeID = node.nextID;
		node.nextID = TreeNode.NULL_TREE_NODE;
		node.height = 0;
		nodeCount++;

		return id;
	}

	function releaseNode(nodeID : Int) {
		Assert.t(nodeCount > 0);
		Assert.t(nodeID >= 0 && nodeID < allocatedNodeCount);
		Assert.t(nodes.get(nodeID).height >= 0);
		var node = nodes.get(nodeID);
		node.nextID = freeID;
		node.height = -1;
		freeID = nodeID;
		nodeCount--;
	}

	inline function addObjectInternal( aabb : AABB ) : Int {
		var nodeID = allocNode();
		Assert.t(nodeID >= 0);

		var gap = aabb.getExtent() * aabbInflate * 0.5;
		var node = nodes.get(nodeID);
		node.aabb.min.load(aabb.min - gap);
		node.aabb.max.load(aabb.max + gap);

		node.height = 0;

		insertLeafNode(nodeID);
		Assert.t(nodes.get(nodeID).isLeaf());

		return nodeID;
	}

	public function removeBody( nodeID : Int ) {
		Assert.t(nodeID >= 0 && nodeID < allocatedNodeCount);
		Assert.t(nodes.get(nodeID).isLeaf());
		removeLeafNode(nodeID);
		releaseNode(nodeID);
	}

	public inline function updateBody( nodeID : Int, aabb : AABB, forceReinsert : Bool ) {
		Assert.t(nodeID >= 0 && nodeID < allocatedNodeCount);
		Assert.t(nodes.get(nodeID).isLeaf());
		Assert.t(nodes.get(nodeID).height >= 0);

		if ( !forceReinsert && nodes.get(nodeID).aabb.containsAABB(aabb) )
			return false;

		removeLeafNode(nodeID);

		var node = nodes.get(nodeID);
		node.aabb = aabb;
		var gap = aabb.getExtent() * aabbInflate * 0.5;
		node.aabb.min.load(aabb.min - gap);
		node.aabb.max.load(aabb.max + gap);

		Assert.t(nodes.get(nodeID).aabb.containsAABB(aabb));

		insertLeafNode(nodeID);

		return true;
	}

	function insertLeafNode(nodeID : Int) {
		if ( rootID == TreeNode.NULL_TREE_NODE ) {
			rootID = nodeID;
			nodes.get(rootID).nextID = TreeNode.NULL_TREE_NODE;
			return;
		}

		Assert.t(rootID != TreeNode.NULL_TREE_NODE);

		var node = nodes.get(nodeID);
		var newNodeAABB = node.aabb.clone();
		var curID = rootID;
		while( !nodes.get(curID).isLeaf() ) {
			var curNode = nodes.get(curID);
			var leftChild = curNode.leftChild;
			var rightChild = curNode.rightChild;

			var volumeAABB = curNode.aabb.getVolume();
			var mergedAABBs = new AABB(new Vec3(), new Vec3());
			mergedAABBs.mergeTwo(curNode.aabb, newNodeAABB);
			var mergeVolume = mergedAABBs.getVolume();

			var costS = 2.0 * mergeVolume;
			var costI = 2.0 * (mergeVolume - volumeAABB);

			var costLeft : Scalar;
			var leftNode = nodes.get(leftChild);
			var currentAndLeftAABB = new AABB(new Vec3(), new Vec3());
			currentAndLeftAABB.mergeTwo(newNodeAABB, leftNode.aabb);
			if( leftNode.isLeaf() ) {
				costLeft = currentAndLeftAABB.getVolume() + costI;
			} else {
				var leftChildVolume = leftNode.aabb.getVolume();
				costLeft = costI + currentAndLeftAABB.getVolume() - leftChildVolume;
			}

			var costRight : Scalar;
			var rightNode = nodes.get(rightChild);
			var currentAndRightAABB = new AABB(new Vec3(), new Vec3());
			currentAndRightAABB.mergeTwo(newNodeAABB, rightNode.aabb);
			if ( rightNode.isLeaf() ) {
				costRight = currentAndRightAABB.getVolume() + costI;
			} else {
				var rightChildVolume = rightNode.aabb.getVolume();
				costRight = costI + currentAndRightAABB.getVolume() - rightChildVolume;
			}

			if ( costS < costLeft && costS < costRight )
				break;

			curID = costLeft < costRight ? leftChild : rightChild;
		}

		var siblingNodeID = curID;

		var newParentNodeID = allocNode();
		var newParentNode = nodes.get(newParentNodeID);

		var siblingNode = nodes.get(siblingNodeID);
		var oldParentNodeID = siblingNode.nextID;

		// We must reget node since allocNode may have realloc the array of nodes
		node = nodes.get(nodeID);

		newParentNode.nextID = oldParentNodeID;
		newParentNode.aabb.mergeTwo(siblingNode.aabb, newNodeAABB);
		newParentNode.height = siblingNode.height + 1;
		Assert.t(newParentNode.height > 0);

		if ( oldParentNodeID != TreeNode.NULL_TREE_NODE ) {
			var oldParentNode = nodes.get(oldParentNodeID);
			Assert.t(!oldParentNode.isLeaf());
			if ( oldParentNode.leftChild == siblingNodeID )
				oldParentNode.leftChild = newParentNodeID;
			else
				oldParentNode.rightChild = newParentNodeID;
			newParentNode.leftChild = siblingNodeID;
			newParentNode.rightChild = nodeID;
			siblingNode.nextID = newParentNodeID;
			node.nextID = newParentNodeID;
		} else {
			newParentNode.leftChild = siblingNodeID;
			newParentNode.rightChild = nodeID;
			siblingNode.nextID = newParentNodeID;
			node.nextID = newParentNodeID;
			rootID = newParentNodeID;
		}
		curID = node.nextID;
		Assert.t(!nodes.get(curID).isLeaf());
		while ( curID != TreeNode.NULL_TREE_NODE ) {
			curID = balanceSubTreeAtNode(curID);
			var curNode = nodes.get(curID);
			Assert.t(nodes.get(nodeID).isLeaf());
			Assert.t(!nodes.get(curID).isLeaf());
			var leftChild = curNode.leftChild;
			var rightChild = curNode.rightChild;
			Assert.t(leftChild != TreeNode.NULL_TREE_NODE);
			Assert.t(rightChild != TreeNode.NULL_TREE_NODE);

			curNode.height = Math.imax(nodes.get(leftChild).height, nodes.get(rightChild).height) + 1;

			Assert.t( curNode.height > 0 );

			curNode.aabb.mergeTwo(nodes.get(leftChild).aabb, nodes.get(rightChild).aabb);
			curID = nodes.get(curID).nextID;
		}

		Assert.t(nodes.get(nodeID).isLeaf());
	}

	function removeLeafNode( nodeID : Int ) {
		Assert.t(nodeID >= 0 && nodeID < allocatedNodeCount);
		Assert.t(nodes.get(nodeID).isLeaf());

		if ( rootID == nodeID ) {
			rootID = TreeNode.NULL_TREE_NODE;
			return;
		}

		var node = nodes.get(nodeID);
		var parentID = node.nextID;
		var parentNode = nodes.get(parentID);
		var grandParentNodeID = parentNode.nextID;
		var siblingNodeID = parentNode.leftChild == nodeID ? parentNode.rightChild : parentNode.leftChild;
		var siblingNode = nodes.get(siblingNodeID);

		if ( grandParentNodeID != TreeNode.NULL_TREE_NODE ) {
			var grandParentNode = nodes.get(grandParentNodeID);
			if ( grandParentNode.leftChild == parentID )
				grandParentNode.leftChild = siblingNodeID;
			else {
				Assert.t( grandParentNode.rightChild == parentID );
				grandParentNode.rightChild = siblingNodeID;
			}

			siblingNode.nextID = grandParentNodeID;
			releaseNode(parentID);

			var curID = grandParentNodeID;
			while ( curID != TreeNode.NULL_TREE_NODE ) {
				curID = balanceSubTreeAtNode(curID);

				Assert.t(!nodes.get(curID).isLeaf());

				var curNode = nodes.get(curID);
				var leftChild = curNode.leftChild;
				var rightChild = curNode.rightChild;

				var leftNode = nodes.get(leftChild);
				var rightNode = nodes.get(rightChild);
				curNode.aabb.mergeTwo(leftNode.aabb, rightNode.aabb);
				curNode.height = Math.imax(leftNode.height, rightNode.height) + 1;
				Assert.t(curNode.height > 0);

				curID = nodes.get(curID).nextID;
			}
		} else {
			rootID = siblingNodeID;
			siblingNode.nextID = TreeNode.NULL_TREE_NODE;
			releaseNode(parentID);
		}
	}

	function balanceSubTreeAtNode( nodeID : Int ) : Int {
		Assert.t(nodeID != TreeNode.NULL_TREE_NODE);

		var nodeA = nodes.get(nodeID);

		if ( nodeA.isLeaf() || nodeA.height < 2 )
			return nodeID;

		var nodeBID = nodeA.leftChild;
		var nodeCID = nodeA.rightChild;

		inline function assertID(nodeID : Int) { Assert.t(nodeID >= 0 || nodeID < allocatedNodeCount); }
		assertID(nodeBID);
		assertID(nodeCID);

		var nodeB = nodes.get(nodeBID);
		var nodeC = nodes.get(nodeCID);

		var balanceFactor = nodeC.height - nodeB.height;

		if ( balanceFactor > 1 ) {
			Assert.t( !nodeC.isLeaf() );

			var nodeFID = nodeC.leftChild;
			var nodeGID = nodeC.rightChild;
			assertID(nodeFID);
			assertID(nodeGID);
			var nodeF = nodes.get(nodeFID);
			var nodeG = nodes.get(nodeGID);

			nodeC.leftChild = nodeID;
			nodeC.nextID = nodeA.nextID;
			nodeA.nextID = nodeCID;

			if( nodeC.nextID != TreeNode.NULL_TREE_NODE ) {

				if ( nodes.get(nodeC.nextID).leftChild == nodeID )
					nodes.get(nodeC.nextID).leftChild = nodeCID;
				else {
					Assert.t( nodes.get(nodeC.nextID).rightChild == nodeID );
					nodes.get(nodeC.nextID).rightChild = nodeCID;
				}
			} else
				rootID = nodeCID;

			Assert.t(!nodeC.isLeaf());
			Assert.t(!nodeA.isLeaf());

			if ( nodeF.height > nodeG.height ) {

				nodeC.rightChild = nodeFID;
				nodeA.rightChild = nodeGID;
				nodeG.nextID = nodeID;

				nodeA.aabb.mergeTwo(nodeB.aabb, nodeG.aabb);
				nodeC.aabb.mergeTwo(nodeA.aabb, nodeF.aabb);

				nodeA.height = Math.imax(nodeB.height, nodeG.height) + 1;
				nodeC.height = Math.imax(nodeA.height, nodeF.height) + 1;
				Assert.t(nodeA.height > 0);
				Assert.t(nodeC.height > 0);
			} else {

				nodeC.rightChild = nodeGID;
				nodeA.rightChild = nodeFID;
				nodeF.nextID = nodeID;

				nodeA.aabb.mergeTwo(nodeB.aabb, nodeF.aabb);
				nodeC.aabb.mergeTwo(nodeA.aabb, nodeG.aabb);

				nodeA.height = Math.imax(nodeB.height, nodeF.height) + 1;
				nodeC.height = Math.imax(nodeA.height, nodeG.height) + 1;
				Assert.t(nodeA.height > 0);
				Assert.t(nodeC.height > 0);
			}

			return nodeCID;
		}

		if ( balanceFactor < -1 ) {
			Assert.t(!nodeB.isLeaf());

			var nodeFID = nodeB.leftChild;
			var nodeGID = nodeB.rightChild;
			assertID(nodeFID);
			assertID(nodeGID);
			var nodeF = nodes.get(nodeFID);
			var nodeG = nodes.get(nodeGID);

			nodeB.leftChild = nodeID;
			nodeB.nextID = nodeA.nextID;
			nodeA.nextID = nodeBID;

			if ( nodeB.nextID != TreeNode.NULL_TREE_NODE ) {
				if( nodes.get(nodeB.nextID).leftChild == nodeID )
					nodes.get(nodeB.nextID).leftChild = nodeBID;
				else {
					Assert.t(nodes.get(nodeB.nextID).rightChild == nodeID);
					nodes.get(nodeB.nextID).rightChild = nodeBID;
				}
			} else {
				rootID = nodeBID;
			}

			Assert.t(!nodeB.isLeaf());
			Assert.t(!nodeA.isLeaf());

			if(nodeF.height > nodeG.height) {
				nodeB.rightChild = nodeFID;
				nodeA.leftChild = nodeGID;
				nodeG.nextID = nodeID;

				nodeA.aabb.mergeTwo(nodeC.aabb, nodeG.aabb);
				nodeB.aabb.mergeTwo(nodeA.aabb, nodeF.aabb);

				nodeA.height = Math.imax(nodeC.height, nodeG.height) + 1;
				nodeB.height = Math.imax(nodeA.height, nodeF.height) + 1;
				Assert.t(nodeA.height > 0);
				Assert.t(nodeB.height > 0);
			} else {
				nodeB.rightChild = nodeGID;
				nodeA.leftChild = nodeFID;
				nodeF.nextID = nodeID;

				nodeA.aabb.mergeTwo(nodeC.aabb, nodeF.aabb);
				nodeB.aabb.mergeTwo(nodeA.aabb, nodeG.aabb);

				nodeA.height = Math.imax(nodeC.height, nodeF.height) + 1;
				nodeB.height = Math.imax(nodeA.height, nodeG.height) + 1;
				Assert.t(nodeA.height > 0);
				Assert.t(nodeB.height > 0);
			}

			return nodeBID;
		}

		return nodeID;
	}

	public function getTreeNode( nodeID : Int ) : TreeNode {
		return nodes.get(nodeID);
	}

	@:generic
	public function walkTree<T : TreeVisitor>( visitor : T ) : Void {
		if ( nodeCount == 0 )
			return;
		if ( visitor.inUse )
			throw "Can't use already in-use Visitor!";
		visitor.inUse = true;
		var stack = visitor.stack;
		if ( stack == null || stack.length < nodeCount ) {
			stack = new hl.NativeArray<Int>(nodeCount);
			visitor.stack = stack;
		}

		stack[0] = rootID;
		var cursor = 1;
		while ( cursor > 0 ) {
			var id = stack[--cursor];
			if ( id == TreeNode.NULL_TREE_NODE )
				continue;

			var nodeToVisit = nodes.get(id);

			if ( nodeToVisit.isLeaf() ) {
				if ( !visitor.visitBody(nodeToVisit) )
					break;
			} else if ( visitor.visitNode(nodeToVisit) ) {
				stack[cursor++] = nodeToVisit.leftChild;
				stack[cursor++] = nodeToVisit.rightChild;
			}
		}
		visitor.inUse = false;
	}

	#if heaps
	var debugVisitor : DebugVisitor;
	public function makeDebugObj() : h3d.scene.Object {
		if( debugVisitor == null )
			debugVisitor = new DebugVisitor();
		debugVisitor.init();
		walkTree(debugVisitor);
		return debugVisitor.root;
	}
	#end
}