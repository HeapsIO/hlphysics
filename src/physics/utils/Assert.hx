package physics.utils;

class Assert {

	// condition is guarded by macro, so msg / pos from the caller are constructed only on failure
	public static macro function t( condition : haxe.macro.Expr ) : haxe.macro.Expr {
		#if physics_debug
		var pos = haxe.macro.Context.currentPos();
		var failure = macro physics.utils.Assert.tImpl();
		failure.pos = pos;
		var result = macro if( !$condition ) $failure;
		result.pos = pos;
		return result;
		#else
		return macro {};
		#end
	}

	public static macro function w( condition : haxe.macro.Expr, msg : haxe.macro.Expr ) : haxe.macro.Expr {
		#if physics_debug
		var pos = haxe.macro.Context.currentPos();
		var warning = macro physics.utils.Assert.wImpl($msg);
		warning.pos = pos;
		var result = macro if( !$condition ) $warning;
		result.pos = pos;
		return result;
		#else
		return macro {};
		#end
	}

	#if !macro
	public static inline function tImpl( ?pos : haxe.PosInfos ) : Void {
		throw "[ASSERT] " + pos.className + "." + pos.methodName + "(" + pos.fileName + ":" + pos.lineNumber + ")";
	}

	public static inline function wImpl( msg : String, ?pos : haxe.PosInfos ) : Void {
		trace("[WARNING] " + msg + " " + pos.className + "." + pos.methodName + "(" + pos.fileName + ":" + pos.lineNumber + ")");
	}
	#end
}
