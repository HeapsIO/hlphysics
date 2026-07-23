class MyProfiler extends physics.utils.Profiler {

	var s2d : h2d.Scene;
	var bench : h3d.impl.Benchmark;

	public function new(s2d : h2d.Scene) {
		this.s2d = s2d;
	}

	public function update() {
		if( hxd.Key.isPressed(hxd.Key.F6)) {
			if(bench == null) {
				bench = new h3d.impl.Benchmark();
				s2d.add(bench, 10);
				bench.enable = true;
				bench.measureCpu = true;
			} else {
				bench.clear();
				bench.remove();
				bench = null;
			}
		}
		if ( bench != null ) {
			bench.setPosition(0, s2d.height - bench.height);
			bench.begin();
		}
	}

	public function mark( name : String ) {
		if ( bench != null )
			bench.measure(name);
	}
}
