package physics.utils;

@:generic class StaticArrayIterator<T> {
	public var i(default, null) : Int;
	var l : Int;
	var a : StaticArray<T>;
	public inline function new(a, i) {
		this.i = i;
		this.a = a;
		this.l = this.a.length;
	}
	public inline function hasNext() {
		return i < l;
	}
	public inline function next() : T {
		return a.get(i++);
	}
}

@:generic class StaticArray<T> {
	var container : hl.CArray<T>;
	var _length : haxe.atomic.AtomicInt;
	public var length(get, never) : Int;
	inline function get_length() : Int{
		return _length.load();
	}
	public var capacity(default, null) : Int;
	var cl : Class<T>;

	public function new( cl : Class<T>, capacity : Int ) {
		container = hl.CArray.alloc(cl, capacity);
		_length = new haxe.atomic.AtomicInt(0);
		this.capacity = capacity;
		this.cl = cl;
	}

	public function resize( newCapacity : Int ) {
		if( newCapacity <= capacity )
			throw "Invalid new size, wants " + newCapacity + " <= " + capacity;
		var oldContainer = container;
		capacity = newCapacity;
		container = hl.CArray.alloc(cl, newCapacity);
		#if (hl_ver < version("1.16.0"))
		for ( i in 0...length )
			container.unsafeSet(i, oldContainer[i]);
		#else
		container.blit(cl, 0, oldContainer, 0, length);
		#end
	}

	public function pushEmpty() {
		if ( isFull() )
			resize(capacity << 1);
		var idx = _length.add(1);
		return container[idx];
	}

	public function push( value : T ) {
		if ( isFull() )
			resize(capacity << 1);
		set(_length.add(1), value);
	}

	public function pop() : T {
		if ( _length.load() == 0 )
			return null;
		return container[_length.sub(1) - 1];
	}

	public inline function empty() {
		_length.store(0);
	}

	public inline function isFull() {
		return _length.load() == capacity;
	}

	public inline function get( pos : Int ) : T {
		Assert.t(pos >= 0 && pos < _length.load());
		return container[pos];
	}

	public inline function getLast() : T {
		var len = _length.load();
		if( len == 0 )
			return null;
		return container[len - 1];
	}

	public inline function set( pos : Int, value : T ) : T {
		Assert.t(pos >= 0 && pos < _length.load());
		return container.unsafeSet(pos, value);
	}

	public inline function iterator() : StaticArrayIterator<T> {
		return new StaticArrayIterator(this, 0);
	}
}