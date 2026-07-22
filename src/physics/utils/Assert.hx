package physics.utils;

class Assert {
	public static inline function t( condition : Bool, ?pos : haxe.PosInfos ) : Void {
		#if physics_debug
		if( !condition )
			throw "assert " + pos.className + "." + pos.methodName + "(" + pos.fileName + ":" + pos.lineNumber + ")";
		#end
	}

	public static inline function w( condition : Bool, msg : String, ?pos : haxe.PosInfos ) : Void {
		#if physics_debug
		if( !condition )
			trace("[WARNING] " + msg + " " + pos.className + "." + pos.methodName + "(" + pos.fileName + ":" + pos.lineNumber + ")");
		#end
	}
}
