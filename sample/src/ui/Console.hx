package ui;

@:rtti
@:keep
class Console extends h2d.Console {
	public function registerCommands(o: Dynamic) {
		function regRec(cl: Dynamic) {
			if(!haxe.rtti.Rtti.hasRtti(cl))
				return;
			var rtti = haxe.rtti.Rtti.getRtti(cl);
			function regCmd(field: haxe.rtti.CType.ClassField, args: Array<haxe.rtti.CType.FunctionArgument>) {
				var cargs = [];
				for(a in args) {
					var nt : h2d.Console.ConsoleArg = switch a.t {
						case CClass("String", _): AString;
						case CAbstract("Int", _): AInt;
						case CAbstract("Float", _): AFloat;
						default:
							return;
					}
					cargs.push({
						name: a.name,
						opt: a.opt,
						t: nt
					});
				}

				var func = Reflect.field(o, field.name);
				function doCall(args: Array<Dynamic>) {
					Reflect.callMethod(o, func, args);
				}
				addCommand(field.name, field.doc, cargs, Reflect.makeVarArgs(doCall));
			}

			for(field in rtti.fields) {
				var cmd = null;
				for( m in field.meta ) {
					if( m.name == "cmd" ) {
						cmd = m;
						break;
					}
				}
				if(cmd != null) {
					switch field.type {
						case CFunction(args, ret):
							regCmd(field, args);
							if(cmd.params.length == 1) {
								var alias = StringTools.trim(StringTools.replace(cmd.params[0], "\"", ""));
								addAlias(alias, field.name);
							}
						default:
					}
				}
			}
		}

		var cl = Type.getClass(o);
		while(cl != null) {
			regRec(cl);
			cl = Type.getSuperClass(cl);
		}
	}

	public function new(font:h2d.Font,?parent) {
		super(font, parent);
		registerCommands(this);
	}

	static var profilingCPU : Bool = false;
	static function profStart() {
		hl.Profile.event(-7,"10000"); // setup
		hl.Profile.event(-3); // clear data
		hl.Profile.event(-5); // resume all
		profilingCPU = true;
		return true;
	}

	static function profDump() {
		profilingCPU = false;
		hl.Profile.event(-6); // save dump
		hl.Profile.event(-4); // pause all
		hl.Profile.event(-3); // clear data
		#if hashlink
		try {
			hlprof.ProfileGen.run(["-o", "hlprofile.json"]);
		} catch( e ) {
			return false;
		}
		#end
		return true;
	}

	@cmd function prof(seconds : Float = -1.0) {
		// https://github.com/HaxeFoundation/hashlink/wiki/Profiler
		inline function start() {
			if ( profStart() )
				log("Profiling started");
			else
				log("Could not start profiling");
		}
		inline function dump() {
			if ( profDump() )
				log("Dump hlprofile.dump");
			else
				log("Could not post process profile dump : missing profiler.hl compilation?");
		}
		if ( seconds > 0.0 ) {
			start();
			haxe.Timer.delay(function() {
				dump();
			}, hxd.Math.round(seconds * 1000.0));
			hide();
			return;
		} else {
			if ( !profilingCPU ) {
				start();
				log("'/prof' again to dump.");
			} else {
				dump();
			}
			hide();
			return;
		}
	}

	@cmd function memprof(cmd) {
		switch (cmd) {
		case "start":
			var tmp = hl.Profile.globalBits;
			tmp.set(Alloc);
			hl.Profile.globalBits = tmp;
			hl.Profile.reset();
		case "dump":
			hl.Profile.dump("memprofSize.dump", true, false);
			hl.Profile.dump("memprofCount.dump", false, true);
		}
		hide();
	}
}
