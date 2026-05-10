import asyncio
import threading
import atexit
import warnings
from typing import Any, Callable, Dict, Set

class GlobalEventHandler:
    """Central async event bus for all @event.when_xxx handlers."""
    _instance = None
    _loop: asyncio.AbstractEventLoop | None = None
    _thread: threading.Thread | None = None

    # Regular when_xxx events
    _handlers: Dict[str, Set[Callable]] = {}
    # New: task-specific handlers (keyed by the actual Tasks enum member)
    _task_handlers: Dict[Any, Set[Callable]] = {}

    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    @classmethod
    def start(cls):
        with cls._lock:
            if cls._loop is not None:
                return
            cls._loop = asyncio.new_event_loop()
            cls._thread = threading.Thread(
                target=cls._run_loop,
                daemon=True,
                name="GlobalEventHandler"
            )
            cls._thread.start()

    @classmethod
    def _run_loop(cls):
        asyncio.set_event_loop(cls._loop)
        cls._loop.run_forever()

    @classmethod
    def shutdown(cls):
        """Clean shutdown to prevent core dump."""
        with cls._lock:
            if cls._loop is None:
                return
            try:
                cls._loop.call_soon_threadsafe(cls._loop.stop)
            except RuntimeError:
                pass
            if cls._thread is not None:
                cls._thread.join(timeout=2.0)

    @classmethod
    def register(cls, event_name: str, handler: Callable):
        cls._handlers.setdefault(event_name, set()).add(handler)

    @classmethod
    def emit(cls, event_name: str, *args, **kwargs):
        """Thread-safe emit from any thread (ROS callbacks, main thread, etc.)."""
        if cls._loop is None or event_name not in cls._handlers:
            return

        def run_emit():
            for handler in list(cls._handlers[event_name]):
                asyncio.create_task(handler(*args, **kwargs))

        cls._loop.call_soon_threadsafe(run_emit)
        
    @classmethod
    def register_task(cls, task: Any, handler: Callable):
        """Register a handler for a specific scheduler task output."""
        cls._task_handlers.setdefault(task, set()).add(handler)

    @classmethod
    def emit_task(cls, task: Any, *args, **kwargs):
        """Emit a task-output event (called from scheduler)."""
        if cls._loop is None or task not in cls._task_handlers:
            return

        def run_emit():
            for handler in list(cls._task_handlers[task]):
                asyncio.create_task(handler(*args, **kwargs))

        cls._loop.call_soon_threadsafe(run_emit)


class EventNamespace:
    """Global events: @event.when_play, @event.when_tasks(...), etc."""

    def __getattr__(self, name: str):
        if name.startswith("when_"):
            def decorator(func: Callable):
                if not asyncio.iscoroutinefunction(func):
                    raise TypeError(f"@event.{name} handler must be async def")
                GlobalEventHandler.register(name, func)
                return func
            return decorator
        raise AttributeError(f"event has no attribute '{name}'")

    def when_tasks(self, task: Any):
        """Decorator for task scheduler outputs.

        Usage:
            @event.when_tasks(CompanionTasks.GENERATE_COORDS)
            async def handler(scheduler: TaskScheduler, output: Any):
                ...

        The referenced task is **automatically added** to the global_task_scheduler
        (lazy initialization via the proxy). You no longer need to call
        global_task_scheduler.add_task(...) manually.
        """
        def decorator(func: Callable):
            if not asyncio.iscoroutinefunction(func):
                raise TypeError("@event.when_tasks handler must be async def")
            
            GlobalEventHandler.register_task(task, func)
            
            # === LAZY LOAD THE TASK AUTOMATICALLY ===
            try:
                # Import inside decorator → no circular import with scheduler.py
                from create3.scheduler.scheduler import global_task_scheduler
                global_task_scheduler.add_task(task)
            except Exception as e:
                warnings.warn(
                    f"Could not auto-add task '{task}' to scheduler: {e}",
                    RuntimeWarning,
                )
            
            return func
        return decorator

    def trigger(self, event_name: str, *args, **kwargs):
        """Fire a global event (for testing or manual triggering).
        Example: event.trigger('when_play', robot)
        """
        if not event_name.startswith("when_"):
            event_name = f"when_{event_name}"
        GlobalEventHandler.emit(event_name, *args, **kwargs)
        
    def play(self):
        self.trigger("when_play")


# Create the global object and start everything
event = EventNamespace()
GlobalEventHandler.start()
atexit.register(GlobalEventHandler.shutdown)