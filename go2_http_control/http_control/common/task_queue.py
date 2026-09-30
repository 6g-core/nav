"""Single-task executor shared by HTTP-controlled robot actions."""

from collections import deque
import threading


class TaskCancelled(RuntimeError):
    """A task was cancelled before completing."""


class TaskHandle:
    """Completion state for one task; completion is owned by the executor."""

    def __init__(self, task):
        self.task = task
        self.done = threading.Event()
        self.cancelled = threading.Event()
        self.error = None

    def _finish(self, error=None):
        # Called only under the executor lock. Cancellation wins over a late
        # task result, without allowing the next task to run prematurely.
        if not self.done.is_set():
            self.error = error
            self.done.set()


class TaskExecutor:
    """Run at most one robot task and reject concurrent submissions."""

    def __init__(self, on_error=None, interval_s=0.1):
        self._queue = deque()
        self._lock = threading.Lock()
        self._active = None
        self._stop_event = threading.Event()
        self._thread = None
        self._on_error = on_error
        self._interval_s = interval_s

    @property
    def busy(self):
        """Whether a task is running or waiting to run."""
        with self._lock:
            return self._active is not None or bool(self._queue)

    def submit(self, task, on_accept=None):
        """Queue one task and return whether it was accepted."""
        return self.submit_handle(task, on_accept=on_accept) is not None

    def submit_handle(self, task, on_accept=None):
        """Return a completion handle, or None when busy or shutting down.

        The callback runs under the queue lock only for accepted tasks. It
        must not block or call back into the executor.
        """
        with self._lock:
            if self._stop_event.is_set() or self._active is not None or self._queue:
                return None
            if on_accept is not None:
                on_accept()
            handle = TaskHandle(task)
            self._queue.append(handle)
            return handle

    def clear_pending(self):
        """Drop tasks that have not started yet and return their count."""
        with self._lock:
            count = len(self._queue)
            for handle in self._queue:
                handle.cancelled.set()
                handle._finish(TaskCancelled('Task cancelled before execution'))
            self._queue.clear()
            return count

    def cancel(self, handle=None, on_cancel=None, error=None):
        """Cancel one task or all tasks and wake their waiters.

        Return the number removed from the queue, or None if a specific task
        already completed. The optional nonblocking callback runs under the
        queue lock, just like on_accept. A running task stays busy until exit.
        """
        with self._lock:
            targets = list(self._queue)
            if self._active is not None:
                targets.append(self._active)
            if handle is not None:
                if handle not in targets or handle.done.is_set():
                    return None
                targets = [handle]
            if on_cancel is not None:
                on_cancel()
            cleared = 0
            for target in targets:
                target.cancelled.set()
                target._finish(error or TaskCancelled('Task cancelled'))
                if target in self._queue:
                    self._queue.remove(target)
                    cleared += 1
            return cleared

    def start(self):
        """Start the worker thread once."""
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="nav-task-executor",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout_s=1.0):
        """Stop the worker thread during node shutdown."""
        self._stop_event.set()
        self.cancel()
        if self._thread:
            self._thread.join(timeout=timeout_s)

    def _run(self):
        while not self._stop_event.is_set():
            handle = None
            with self._lock:
                if self._queue:
                    handle = self._queue.popleft()
                    self._active = handle

            if handle is not None:
                error = None
                try:
                    if not handle.cancelled.is_set():
                        handle.task()
                except Exception as exc:  # pragma: no cover - hardware path
                    error = exc
                    if self._on_error and not isinstance(exc, TaskCancelled):
                        self._on_error(exc)
                finally:
                    with self._lock:
                        handle._finish(error)
                        self._active = None

            self._stop_event.wait(self._interval_s)
