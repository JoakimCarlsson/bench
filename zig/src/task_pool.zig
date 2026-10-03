//! The engine's task pool: spinning workers woken by an epoch counter, a
//! fork-join `run` and a block-claiming `parallelFor`.
const std = @import("std");

const Allocator = std.mem.Allocator;
const Atomic = std.atomic.Value;

const TaskPool = @This();

const spin_iterations: u32 = 4000;

const Call = *const fn (*anyopaque, u32) void;

io: std.Io,
workers: []std.Thread,
epoch: Atomic(u32) = .init(0),
finished: Atomic(u32) = .init(0),
stopping: Atomic(bool) = .init(false),
call: ?Call = null,
data: ?*anyopaque = null,

/// Starts `thread_count - 1` workers; the caller is worker 0. The pool lives
/// on the heap so the workers can hold its address.
pub fn create(gpa: Allocator, io: std.Io, thread_count: u32) !*TaskPool {
    const self = try gpa.create(TaskPool);
    errdefer gpa.destroy(self);
    self.* = .{ .io = io, .workers = try gpa.alloc(std.Thread, thread_count - 1) };
    errdefer gpa.free(self.workers);
    for (self.workers, 1..) |*worker, index| worker.* = try std.Thread.spawn(.{}, workerLoop, .{ self, @as(u32, @intCast(index)) });
    return self;
}

/// Stops and joins the workers, then frees the pool.
pub fn destroy(self: *TaskPool, gpa: Allocator) void {
    self.stopping.store(true, .release);
    _ = self.epoch.fetchAdd(1, .release);
    self.io.futexWake(u32, &self.epoch.raw, std.math.maxInt(u32));
    for (self.workers) |worker| worker.join();
    gpa.free(self.workers);
    gpa.destroy(self);
}

/// Workers plus the calling thread.
pub fn threadCount(self: *const TaskPool) u32 {
    return @intCast(self.workers.len + 1);
}

/// A spin-wait hint to the CPU.
pub fn relax() void {
    std.atomic.spinLoopHint();
}

/// Runs `task.work(worker)` on every thread and waits for all of them.
pub fn run(self: *TaskPool, task: anytype) void {
    const Task = @TypeOf(task);
    const Thunk = struct {
        /// Calls the task behind the erased pointer.
        fn call(data: *anyopaque, worker: u32) void {
            const typed: Task = @ptrCast(@alignCast(data));
            typed.work(worker);
        }
    };
    self.dispatch(Thunk.call, @ptrCast(@constCast(task)));
}

/// Runs `task.range(begin, end)` over [0, count) in blocks of `grain`,
/// claimed by whichever thread is free.
pub fn parallelFor(self: *TaskPool, count: usize, grain: usize, task: anytype) void {
    if (count == 0) return;
    const blocks = (count + grain - 1) / grain;
    if (blocks <= 1 or self.workers.len == 0) {
        task.range(0, count);
        return;
    }
    var claimer: BlockClaimer(@TypeOf(task)) = .{ .task = task, .count = count, .grain = grain, .blocks = blocks };
    self.run(&claimer);
}

/// Hands out blocks of a range to whichever worker asks next.
fn BlockClaimer(comptime Task: type) type {
    return struct {
        task: Task,
        count: usize,
        grain: usize,
        blocks: usize,
        next: Atomic(usize) = .init(0),

        /// Claims and runs blocks until none are left.
        pub fn work(self: *@This(), worker: u32) void {
            _ = worker;
            while (true) {
                const block = self.next.fetchAdd(1, .monotonic);
                if (block >= self.blocks) return;
                const begin = block * self.grain;
                self.task.range(begin, @min(self.count, begin + self.grain));
            }
        }
    };
}

/// Spins, then sleeps, until a new epoch, and runs the published call.
fn workerLoop(self: *TaskPool, index: u32) void {
    var seen: u32 = 0;
    while (true) {
        var spins: u32 = 0;
        while (self.epoch.load(.acquire) == seen) {
            spins +|= 1;
            if (spins < spin_iterations) {
                relax();
            } else {
                self.io.futexWaitUncancelable(u32, &self.epoch.raw, seen);
            }
        }
        seen = self.epoch.load(.acquire);
        if (self.stopping.load(.acquire)) return;
        self.call.?(self.data.?, index);
        _ = self.finished.fetchAdd(1, .release);
    }
}

/// Publishes a call to the workers, runs it here as worker 0, and waits.
fn dispatch(self: *TaskPool, call: Call, data: *anyopaque) void {
    if (self.workers.len == 0) {
        call(data, 0);
        return;
    }
    self.call = call;
    self.data = data;
    self.finished.store(0, .monotonic);
    _ = self.epoch.fetchAdd(1, .release);
    self.io.futexWake(u32, &self.epoch.raw, std.math.maxInt(u32));
    call(data, 0);
    const expected: u32 = @intCast(self.workers.len);
    while (self.finished.load(.acquire) != expected) relax();
}
