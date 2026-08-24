package physics.utils;

class Assert {
	public static macro function t( condition : haxe.macro.Expr ) : haxe.macro.Expr {
		#if physics_debug
		return macro physics.utils.Assert.tImpl($condition);
		#else
		return macro {};
		#end
	}

	public static macro function w( condition : haxe.macro.Expr, msg : haxe.macro.Expr ) : haxe.macro.Expr {
		#if physics_debug
		return macro physics.utils.Assert.wImpl($condition, $msg);
		#else
		return macro {};
		#end
	}

	#if !macro
	public static inline function tImpl( condition : Bool, ?pos : haxe.PosInfos ) : Void {
		if( !condition )
			throw "[ASSERT] " + pos.className + "." + pos.methodName + "(" + pos.fileName + ":" + pos.lineNumber + ")";
	}

	public static inline function wImpl( condition : Bool, msg : String, ?pos : haxe.PosInfos ) : Void {
		if( !condition )
			trace("[WARNING] " + msg + " " + pos.className + "." + pos.methodName + "(" + pos.fileName + ":" + pos.lineNumber + ")");
	}
	#end
}
