import asyncio
import traceback
from asyncio import get_event_loop

from unilabos.utils.log import error


def run_async_func(func, *, loop=None, trace_error=True, **kwargs):
    if loop is None:
        loop = get_event_loop()

    def _handle_future_exception(fut):
        try:
            fut.result()
        except Exception as e:
            error(f"异步任务 {func.__name__} 报错了")
            error(traceback.format_exc())

    future = asyncio.run_coroutine_threadsafe(func(**kwargs), loop)
    if trace_error:
        future.add_done_callback(_handle_future_exception)
    return future
