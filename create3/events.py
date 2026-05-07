import asyncio
import threading
import atexit
from typing import Callable, Dict, Set

class GlobalEventHandler:
    """Central async event bus for all @event.when_xxx handlers."""
    _instance = None
    _loop: asyncio.AbstractEventLoop | None = None
    _thread: threading.Thread | None = None
    _handlers: Dict[str, Set[Callable]] = {}
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
            # Simple and clean: one task per handler (no gather needed)
            for handler in list(cls._handlers[event_name]):
                asyncio.create_task(handler(*args, **kwargs))

        cls._loop.call_soon_threadsafe(run_emit)


class EventNamespace:
    """Global events: @event.when_play"""
    def __getattr__(self, name: str):
        if name.startswith("when_"):
            def decorator(func: Callable):
                if not asyncio.iscoroutinefunction(func):
                    raise TypeError(f"@event.{name} handler must be async def")
                GlobalEventHandler.register(name, func)
                return func
            return decorator
        raise AttributeError(f"event has no attribute '{name}'")

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