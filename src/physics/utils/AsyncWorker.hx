package physics.utils;

class AsyncWorker {
	static var instance(get, null) : AsyncWorker;
	static var refCount = 0;

	static function get_instance() : AsyncWorker {
		if ( instance == null )
			instance = new AsyncWorker();
		return instance;
	}

	var thread : sys.thread.Thread;
	final queue : sys.thread.Deque<Void->Void>;

	function new() {
		queue = new sys.thread.Deque();
		thread = sys.thread.Thread.create(loop);
		thread.name = "AsyncWorker";
	}

	function loop() {
		while ( true ) {
			var task = queue.pop(true);
			if ( task == null )
				break;
			task();
		}
	}

	public static function run( task : Void -> Void ) : Void {
		instance.queue.add(task);
	}

	public static function acquire() : Void {
		refCount++;
	}

	public static function release() : Void {
		refCount--;
		if ( refCount <= 0 ) {
			instance.queue.add(null);
			instance = null;
		}
	}
}
