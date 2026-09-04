package physics.collision;

enum CollectorMode {
	All;
	Closest;
	ClosestPerBody;
	Any;
	AnyPerBody;
}

class CollideCollector extends Collector<ContactPoint> {
	public inline function new() {
		super(ContactPoint);
	}

	public inline function addHit( p1 : Vec3, p2 : Vec3, normal : Vec3, penetration : Scalar ) {
		var r : ContactPoint = findSlotToAddHit(-penetration);
		if( r != null ) {
			r.contactPointOn1.load(p1);
			r.contactPointOn2.load(p2);
			r.normal.load(normal);
			r.penetration = penetration;
		}
	}
}

class ShapeCastCollector extends Collector<ShapeCastResult> {
	public inline function new() {
		super(ShapeCastResult);
	}

	public inline function addHit( p1 : Vec3, p2 : Vec3, axis : Vec3, penetration : Scalar, fraction : Scalar ) {
		var r : ShapeCastResult = findSlotToAddHit(fraction == 0.0 ? -penetration : fraction);
		if( r != null ) {
			r.contactPointOn1.load(p1);
			r.contactPointOn2.load(p2);
			r.penetrationAxis.load(axis);
			r.penetration = penetration;
			r.fraction = fraction;
		}
	}
}

class RayCastCollector extends Collector<HitResult> {
	public inline function new() {
		super(HitResult);
	}

	public inline function addHit( position : Vec3, normal : Vec3, fraction : Scalar ) {
		var r : HitResult = findSlotToAddHit(fraction);
		if( r != null ) {
			r.position.load(position);
			r.normal.load(normal);
			r.fraction = fraction;
		}
	}
}

@:generic
abstract class Collector<T> {
	var results : StaticArray<T>;
	var ids : Array<BodyID>;
	var body1Infos : Array<{ id : BodyID, start : Int }>;
	var maxFraction : Scalar;
	public var mode(default, null) : CollectorMode;

	var curId : BodyID;
	public var curMaxFraction(default, null) : Scalar;
	public var curWantsMoreHits(default, null) : Bool;

	public var length(get, never):Int;
	inline function get_length() {
		return results.length;
	}

	#if physics_profile
	var profWorld : PhysicsWorld;
	var profTimes : Map<Shape, Float>;
	var profStart : Float;
	#end

	public function new( cl : Class<T> ) {
		results = new StaticArray(cl, 1);
		ids = [];
		body1Infos = [];
	}

	public function init( maxFraction : Scalar, mode : CollectorMode ) {
		results.empty();
		ids.resize(0);
		body1Infos.resize(0);
		this.maxFraction = maxFraction;
		this.mode = mode;
		curId = -1;
		curMaxFraction = maxFraction;
		curWantsMoreHits = true;
	}

	#if physics_profile
	public function profileStart( world : PhysicsWorld ) {
		profWorld = world;
		if( profTimes == null )
			profTimes = new Map();
	}

	public function profileStop() {
		var res = profTimes;
		profTimes = null;
		profWorld = null;
		return res;
	}
	#end

	public inline function setBody1( id : BodyID ) {
		body1Infos.push({ id : id, start : length });
	}

	public inline function onBody( id : BodyID ) {
		curId = id;
		if( mode == ClosestPerBody )
			curMaxFraction = maxFraction;
		else if( mode == AnyPerBody )
			curWantsMoreHits = true;
		#if physics_profile
		if( profWorld != null )
			profStart = haxe.Timer.stamp();
		#end
	}

	public inline function onBodyEnd() {
		#if physics_profile
		if( profWorld != null ) {
			var elapsed = haxe.Timer.stamp() - profStart;
			var shape = profWorld.getBody(curId).shape;
			var prev = profTimes.get(shape);
			profTimes.set(shape, (prev == null ? 0.0 : prev) + elapsed);
		}
		#end
		curId = -1;
		if( mode == ClosestPerBody )
			curMaxFraction = maxFraction;
		else if( mode == AnyPerBody )
			curWantsMoreHits = true;
	}

	public inline function hasResult() : Bool {
		return results.length > 0;
	}

	public inline function getFirstResult() : { r : Null<T>, b2 : BodyID } {
		var has = hasResult();
		return { r : has ? results.get(0) : null, b2 : has ? ids[0] : -1 };
	}

	public inline function iterResult( callback : (r:T, b2:BodyID) -> Void ) {
		for( i in 0...results.length ) {
			callback(results.get(i), ids[i]);
		}
	}

	public inline function iterBodyIDs( callback : BodyID -> Bool ) {
		for( i in 0...results.length ) {
			if( !callback(ids[i]) )
				break;
		}
	}

	public inline function iterResultsByBody( callback : (arr:StaticArray<T>, start:Int, count:Int, b2:Int) -> Void ) {
		if( results.length > 0 ) {
			var len = results.length;
			var b2 = ids[0];
			var start = 0;
			for( i in 1...len ) {
				var ib2 = ids[i];
				if( ib2 != b2 ) {
					callback(results, start, i-start, b2);
					b2 = ib2;
					start = i;
				}
			}
			callback(results, start, len-start, b2);
		}
	}

	public inline function iterResultsByPair( callback : (arr:StaticArray<T>, start:Int, count:Int, b1:Int, b2:Int) -> Void ) {
		var b1Id = -1;
		var start = 0;
		var b1Ids = [];
		for( b1 in body1Infos ) {
			for( i in start...b1.start )
				b1Ids.push(b1Id);
			b1Id = b1.id;
			start = b1.start;
		}
		for( i in start...results.length )
			b1Ids.push(b1Id);
		if( results.length > 0 ) {
			var len = results.length;
			var b1 = b1Ids[0];
			var b2 = ids[0];
			var start = 0;
			for( i in 1...len ) {
				var ib1 = b1Ids[i];
				var ib2 = ids[i];
				if( b1 != ib1 || ib2 != b2 ) {
					callback(results, start, i-start, b1, b2);
					b1 = ib1;
					b2 = ib2;
					start = i;
				}
			}
			callback(results, start, len-start, b1, b2);
		}
	}

	inline function findSlotToAddHit( fraction : Scalar ) : Null<T> {
		var r : Null<T> = null;
		switch( mode ) {
		case All:
			r = results.pushEmpty();
			ids.push(curId);
		case Closest:
			if( fraction < curMaxFraction ) {
				curMaxFraction = fraction;
				if( results.length == 0 ) {
					r = results.pushEmpty();
				} else {
					r = results.getLast();
				}
				ids[0] = curId;
			}
		case ClosestPerBody:
			if( fraction < curMaxFraction ) {
				curMaxFraction = fraction;
				var lastId = ids.pop();
				if( lastId == null || lastId != curId ) {
					r = results.pushEmpty();
					if( lastId != null )
						ids.push(lastId);
				} else {
					r = results.getLast();
				}
				ids.push(curId);
			}
		case Any:
			if( results.length == 0 ) {
				r = results.pushEmpty();
				ids.push(curId);
			}
			curWantsMoreHits = false;
		case AnyPerBody:
			if( ids.length == 0 || ids[ids.length - 1] != curId ) {
				r = results.pushEmpty();
				ids.push(curId);
			}
			curWantsMoreHits = false;
		}
		return r;
	}
}
