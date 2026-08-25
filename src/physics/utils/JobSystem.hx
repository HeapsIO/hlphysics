package physics.utils;

private class JobWorker {
	var thread : sys.thread.Thread;
	var workerID : Int;
	var jobFinished : haxe.atomic.AtomicInt;
	final queue: sys.thread.Deque<Int->Void>;

	public function new( queue : sys.thread.Deque<Int->Void>, workerID : Int, jobFinished : haxe.atomic.AtomicInt ) {
		this.queue = queue;
		this.workerID = workerID;
		this.jobFinished = jobFinished;
		thread = sys.thread.Thread.create(loop);
		thread.name = "JobWorker" + workerID;
	}

	public function shutdown() {
		queue.add(null);
	}

	function loop() {
		while( true ) {
			var task = queue.pop(true);
			if( task == null )
				break;
			task(workerID);
			jobFinished.add(1);
		}
	}
}

class JobSystem {
	public var workerPool(default, null) : Array<JobWorker>;
	public var jobLaunched : Int;
	public var jobFinished : haxe.atomic.AtomicInt;
	final queue = new sys.thread.Deque<Int->Void>();

	public function new( workerCount : Int ) {
		jobLaunched = 0;
		jobFinished = new haxe.atomic.AtomicInt(0);
		workerPool = [for (i in 0...workerCount) new JobWorker(queue, i, jobFinished)];
	}

	public function runJob( fun : Int -> Void ) {
		jobLaunched++;
		queue.add(fun);
	}

	public function waitForJob() {
		while (jobLaunched != jobFinished.load() ) { Sys.sleep(0); }
		jobLaunched = 0;
		jobFinished.store(0);
	}

	public function shutdown() {
		waitForJob();
		for ( w in workerPool )
			w.shutdown();
	}
}
