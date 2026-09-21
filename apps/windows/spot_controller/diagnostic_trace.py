"""Bounded local connection diagnostics; disk I/O never runs on control/UI threads."""
from collections import deque
from concurrent.futures import Future
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import uuid


def default_folder():
    if os.name == 'nt':
        base = Path(os.environ.get('LOCALAPPDATA', Path.home() / 'AppData/Local'))
    elif sys.platform == 'darwin':
        base = Path.home() / 'Library/Caches'
    else:
        base = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache'))
    return base / 'SpotOMG/RobotConnection'


class ConnectionTrace:
    """One serial writer, two rotating JSONL files, and ordered async exports.

    The bounded memory queue drops diagnostics rather than delaying control.
    Drops are recorded explicitly when the writer catches up. Export requests
    share the writer queue so every earlier accepted record is in the snapshot.
    """
    def __init__(self, folder=None, max_file_bytes=512 * 1024, max_pending=512):
        if max_file_bytes < 256 or max_pending < 1:
            raise ValueError('Trace bounds are too small')
        self.folder = Path(folder) if folder is not None else default_folder()
        self.max_file_bytes = max_file_bytes
        self.max_pending = max_pending
        self._condition = threading.Condition()
        self._pending = deque()
        self._thread = None
        self._closed = False
        self._dropped = 0
        self._exports_pending = 0
        self._write_error = None

    def _start_locked(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name='SpotOMG-diagnostics', daemon=True)
            self._thread.start()
        self._condition.notify()

    @staticmethod
    def _row(event, detail):
        row = dict(epoch_ms=round(time.time() * 1000), uptime_ms=round(time.monotonic() * 1000),
                   event=event, detail=detail[:65536])
        if len(detail) > 65536:
            row['truncated_chars'] = len(detail) - 65536
        return row

    def record(self, event, detail):
        row = self._row(event, detail)
        with self._condition:
            if self._closed:
                return False
            if len(self._pending) >= self.max_pending:
                self._dropped += 1
                return False
            self._pending.append(('record', row))
            self._start_locked()
        return True

    def export(self, path=None):
        future = Future()
        with self._condition:
            if self._closed:
                future.set_exception(RuntimeError('진단 기록이 종료되었습니다.'))
            elif self._exports_pending:
                future.set_exception(RuntimeError('진단 로그를 이미 내보내고 있습니다.'))
            else:
                self._exports_pending = 1
                # One export may follow the bounded record queue without
                # waiting for disk space in that queue on the calling thread.
                self._pending.append(('export', (future, Path(path) if path is not None else None)))
                self._start_locked()
        return future

    def close(self, wait=False, timeout=2):
        """Seal the queue, flush accepted records, then exit; UI calls never join."""
        with self._condition:
            self._closed = True
            self._condition.notify()
            thread = self._thread
        if wait and thread is not None and thread is not threading.current_thread():
            thread.join(timeout)

    def _append(self, row):
        encoded = (json.dumps(row, ensure_ascii=False, separators=(',', ':')) + '\n').encode('utf-8')
        # Preserve valid JSONL even for an unexpectedly huge individual event.
        if len(encoded) > self.max_file_bytes:
            row = dict(row, detail='[oversized diagnostic record omitted]', omitted_bytes=len(encoded))
            encoded = (json.dumps(row, ensure_ascii=False, separators=(',', ':')) + '\n').encode('utf-8')
        self.folder.mkdir(parents=True, exist_ok=True)
        current, previous = self.folder / 'current.jsonl', self.folder / 'previous.jsonl'
        size = current.stat().st_size if current.exists() else 0
        if size + len(encoded) > self.max_file_bytes and current.exists():
            current.replace(previous)
        with current.open('ab') as file:
            file.write(encoded)

    def _export(self, path):
        if self._write_error is not None:
            raise OSError('진단 기록 저장 실패: ' + str(self._write_error))
        path = path or Path(tempfile.gettempdir()) / f'SpotOMG-diagnostics-{uuid.uuid4()}.jsonl'
        if path.resolve() in {(self.folder / name).resolve() for name in ('current.jsonl', 'previous.jsonl')}:
            raise ValueError('현재 기록 파일을 내보내기 대상으로 덮어쓸 수 없습니다.')
        temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
        try:
            with temporary.open('xb') as output:
                for name in ('previous.jsonl', 'current.jsonl'):
                    source = self.folder / name
                    if source.exists():
                        output.write(source.read_bytes())
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)
        return path

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._pending or self._closed)
                if not self._pending:
                    return
                kind, value = self._pending.popleft()
                dropped, self._dropped = self._dropped, 0
            try:
                if dropped:
                    self._append(self._row('trace-overflow', f'{dropped} diagnostic records dropped; control unaffected'))
                if kind == 'record':
                    self._append(value)
            except Exception as exc:
                self._write_error = exc
            if kind == 'export':
                future, path = value
                if not future.set_running_or_notify_cancel():
                    with self._condition:
                        self._exports_pending = 0
                    continue
                try:
                    result = self._export(path)
                except Exception as exc:
                    error = exc
                else:
                    error = None
                with self._condition:
                    self._exports_pending = 0
                if error is not None:
                    future.set_exception(error)
                else:
                    future.set_result(result)
