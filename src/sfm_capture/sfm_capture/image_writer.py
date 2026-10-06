"""Bounded, ordered disk writing without blocking the ROS receive callback."""
from queue import Full, Queue
from threading import Thread
import time


class ImageWriter:
    def __init__(self, dataset, convert, queue_size=4):
        if isinstance(queue_size, bool) or not isinstance(queue_size, int) or queue_size < 1:
            raise ValueError('writer_queue_size must be a positive integer')
        self.dataset = dataset
        self.convert = convert
        self.queue = Queue(maxsize=queue_size)
        self.error = None
        self.peak_queue = 0
        self.max_write_sec = 0.0
        self.closed = False
        self.thread = Thread(target=self._run, name='sfm_png_writer', daemon=True)
        self.thread.start()

    def submit(self, message, stamp, received_ns):
        if self.closed:
            raise RuntimeError('PNG writer is closed')
        if self.error:
            raise RuntimeError(f'PNG writer failed: {self.error}') from self.error
        try:
            self.queue.put_nowait((message, stamp, received_ns))
        except Full as error:
            raise RuntimeError('PNG writer queue is full; reduce save_fps or disk load') from error
        self.peak_queue = max(self.peak_queue, self.queue.qsize())

    def _run(self):
        while True:
            item = self.queue.get()
            try:
                if item is None:
                    return
                if self.error is not None:
                    continue
                message, stamp, received_ns = item
                started = time.monotonic()
                self.dataset.save(self.convert(message), stamp, received_ns,
                                  message.header.frame_id, message.encoding)
                self.max_write_sec = max(self.max_write_sec, time.monotonic() - started)
            except Exception as error:
                self.error = error
            finally:
                self.queue.task_done()

    def close(self):
        if not self.closed:
            self.closed = True
            self.queue.put(None)
            self.thread.join()
        if self.error:
            raise RuntimeError(f'PNG writer failed: {self.error}') from self.error
