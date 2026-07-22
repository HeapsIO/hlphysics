package shiro.physics.container;

@:generic class Pool<T> {
	var clnew : Void -> T;
	var v0 : T;
	var v1 : T;
	var v2 : T;
	var v3 : T;
	var count = 0;

	public function new( cl : Class<T>, clnew : Void -> T ) {
		this.clnew = clnew;
		v0 = clnew();
		v1 = clnew();
		v2 = clnew();
		v3 = clnew();
	}

	public inline function get() : T {
		var v = switch(count) {
			case 0: v0;
			case 1: v1;
			case 2: v2;
			case 3: v3;
			default: throw "empty pool";
		}
		++count;
		return v;
	}

	public inline function put( v : T ) {
		--count;
		switch(count) {
			case 0: v0 = v;
			case 1: v1 = v;
			case 2: v2 = v;
			case 3: v3 = v;
			default: throw "full pool";
		}
	}
}
