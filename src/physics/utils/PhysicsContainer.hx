package physics.utils;

typedef ID = Int;

@:generic class PhysicsIterator<T> {
	public var i(default, null) : Int;
	var l : Int;
	var a : PhysicsContainer<T>;
	public inline function new(a, i) {
		this.i = i;
		this.a = a;
		this.l = this.a.length;
	}
	public inline function hasNext() {
		return i < l;
	}
	public inline function next() : T {
		return @:privateAccess a.container[i++];
	}
}

@:generic class PhysicsContainer<T> {
	var container : hl.CArray<T>;
	var idToIndex : hl.NativeArray<Int>;
	var indexToId : hl.NativeArray<ID>;
	var freeIDs : hl.NativeArray<ID>;
	public var length(default, null) : Int;
	public var capacity(default, null) : Int;
	var cl : Class<T>;

	public function new( cl : Class<T>, capacity : Int ) {
		this.cl = cl;
		this.capacity = capacity;
		container = hl.CArray.alloc(cl, capacity);
		idToIndex = new hl.NativeArray<Int>(capacity);
		indexToId = new hl.NativeArray<ID>(capacity);
		freeIDs = new hl.NativeArray<ID>(capacity);
		for( i in 0...capacity ) {
			idToIndex[i] = -1;
			indexToId[i] = -1;
			freeIDs[i] = i;
		}
		length = 0;
	}

	public function resize( newCapacity : Int ) {
		if( newCapacity <= capacity )
			throw "Invalid new size, wants " + newCapacity + " <= " + capacity;
		var oldContainer = container;
		var oldIdToIndex = idToIndex;
		var oldIndexToId = indexToId;
		var oldFreeIDs = freeIDs;
		var oldCapacity = capacity;
		container = hl.CArray.alloc(cl, newCapacity);
		idToIndex = new hl.NativeArray<Int>(newCapacity);
		indexToId = new hl.NativeArray<ID>(newCapacity);
		freeIDs = new hl.NativeArray<ID>(newCapacity);
		#if (hl_ver < version("1.16.0"))
		for ( i in 0...length )
			container.unsafeSet(i, oldContainer[i]);
		#else
		container.blit(cl, 0, oldContainer, 0, length);
		#end
		idToIndex.blit(0, oldIdToIndex, 0, oldCapacity);
		indexToId.blit(0, oldIndexToId, 0, oldCapacity);
		freeIDs.blit(0, oldFreeIDs, 0, oldCapacity);
		for( i in oldCapacity...newCapacity ) {
			idToIndex[i] = -1;
			indexToId[i] = -1;
			freeIDs[i] = i;
		}
		capacity = newCapacity;
	}

	public function create() : ID {
		if ( length >= capacity )
			resize(capacity << 1);
		var index = length++;
		var freeIndex = index;
		var id = freeIDs[freeIndex];
		freeIDs[freeIndex] = -1;
		idToIndex[id] = index;
		indexToId[index] = id;
		return id;
	}

	public function remove( id : ID ) {
		var oldIndex = idToIndex[id];
		if ( oldIndex == -1 )
			throw "Element does not exist.";
		if ( oldIndex >= length )
			throw "assert";
		idToIndex[id] = -1;
		var index = --length;
		freeIDs[index] = id;
		if ( oldIndex != index ) {
			container.unsafeSet(oldIndex, container[index]);
			var idToRefresh = indexToId[index];
			idToIndex[idToRefresh] = oldIndex;
			indexToId[oldIndex] = idToRefresh;
		}
		indexToId[index] = -1;
	}

	public function get( id : ID ) : T {
		var idx = idToIndex[id];
		if ( idx < 0 )
			throw "assert";
		var v = container[idx];
		return v;
	}

	public function at( idx : Int ) : T {
		return container[idx];
	}

	public inline function iterator() : PhysicsIterator<T> {
		return new PhysicsIterator(this, 0);
	}
}
