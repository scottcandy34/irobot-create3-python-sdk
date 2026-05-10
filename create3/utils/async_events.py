import asyncio
from typing import Callable, Dict, Set

class AsyncEventMixin:
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._local_handlers: Dict[str, Set[Callable]] = {}
        self.event = self._create_event_namespace()   # ← gives you robot.event.when_xxx

    def _create_event_namespace(self):
        class NodeEventNamespace:
            def __getattr__(inner_self, name: str):   # inner_self to avoid name clash
                if name.startswith("when_"):
                    def decorator(func: Callable):
                        if not asyncio.iscoroutinefunction(func):
                            raise TypeError(f"@robot.event.{name} must be async def")
                        self._local_handlers.setdefault(name, set()).add(func)
                        return func
                    return decorator
                raise AttributeError(name)
        return NodeEventNamespace()

    async def _emit_local(self, event_name: str, *args, **kwargs):
        """Used internally by your Button class / sensors / scheduler for per-robot events."""
        handlers = self._local_handlers.get(event_name, set())
        if not handlers:
            return
        tasks = [asyncio.create_task(h(self, *args, **kwargs)) for h in handlers]
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    # Optional helper if you want a robot to also emit global events
    async def emit_global(self, event_name: str, *args, **kwargs):
        from create3.events import GlobalEventHandler
        await GlobalEventHandler.emit(event_name, self, *args, **kwargs)