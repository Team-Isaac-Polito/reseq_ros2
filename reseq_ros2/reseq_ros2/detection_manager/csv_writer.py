# csv_writer.py
import csv
import os
import queue
import threading
from typing import Dict, Optional


class CSVWriter:
    def __init__(
        self,
        path: str,
        header: list,
        fsync: bool = True,
        background: bool = False,
        queue_size: int = 1024,
    ):
        self.path = path
        self.header = header
        self.fsync = fsync
        self.background = background
        self.lock = threading.Lock()
        self._file = None
        self._writer = None
        self._open_file()
        # background worker
        self._queue: Optional[queue.Queue] = None
        self._worker: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        if self.background:
            self._queue = queue.Queue(maxsize=queue_size)
            self._worker = threading.Thread(target=self._bg_worker, daemon=True)
            self._worker.start()

    def _open_file(self):
        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        file_exists = os.path.exists(self.path)
        self._file = open(self.path, 'a', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._file, fieldnames=self.header)
        if not file_exists:
            self._writer.writeheader()
            self._file.flush()
            if self.fsync:
                os.fsync(self._file.fileno())

    def append_row(self, row: Dict) -> bool:
        if self.background:
            try:
                self._queue.put(row, block=True, timeout=1.0)
            except queue.Full:
                return False
            return True
        else:
            with self.lock:
                try:
                    self._writer.writerow(row)
                    self._file.flush()
                    if self.fsync:
                        os.fsync(self._file.fileno())
                    return True
                except Exception:
                    # caller will record last_error
                    return False

    def _bg_worker(self):
        while not self._stop_event.is_set():
            try:
                row = self._queue.get(timeout=0.2)
            except Exception:
                continue
            with self.lock:
                try:
                    self._writer.writerow(row)
                    self._file.flush()
                    if self.fsync:
                        os.fsync(self._file.fileno())
                except Exception:
                    pass
                finally:
                    self._queue.task_done()

    def close(self):
        if self.background:
            self._stop_event.set()
            if self._worker:
                self._worker.join(timeout=2.0)
        with self.lock:
            if self._file:
                try:
                    self._file.flush()
                    if self.fsync:
                        os.fsync(self._file.fileno())
                except Exception:
                    pass
                try:
                    self._file.close()
                except Exception:
                    pass
                self._file = None
