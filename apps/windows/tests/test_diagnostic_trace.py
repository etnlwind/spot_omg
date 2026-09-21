import json
import threading
import time

import pytest
from PySide6.QtWidgets import QApplication

from spot_controller import connection
from spot_controller.diagnostic_trace import ConnectionTrace


def rows(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]


def test_export_is_ordered_off_caller_thread_and_rotation_is_bounded(tmp_path):
    trace = ConnectionTrace(tmp_path / 'trace', max_file_bytes=512)
    assert trace._thread is None  # Merely opening a window does not start I/O.
    try:
        trace.record('diagnostic', 'before export')
        snapshot = trace.export(tmp_path / 'first.jsonl')
        trace.record('diagnostic', 'after export')
        assert [row['detail'] for row in rows(snapshot.result(timeout=2))] == ['before export']
        for index in range(60):
            trace.record('diagnostic', f'조인트 {index}')
        exported = trace.export(tmp_path / 'rotated.jsonl').result(timeout=2)
        history = rows(exported)
        assert history[-1]['detail'] == '조인트 59'
        assert all(isinstance(row['epoch_ms'], int) and isinstance(row['uptime_ms'], int) for row in history)
        retained = list((tmp_path / 'trace').glob('*.jsonl'))
        assert {file.name for file in retained} == {'previous.jsonl', 'current.jsonl'}
        assert all(file.stat().st_size <= 512 for file in retained)
        assert exported.read_bytes() == b''.join((tmp_path / 'trace' / name).read_bytes()
                                               for name in ('previous.jsonl', 'current.jsonl'))
    finally:
        trace.close(wait=True)
    assert not trace._thread.is_alive()


def test_slow_disk_bounds_queue_without_blocking_record_export_or_shutdown(tmp_path, monkeypatch):
    trace = ConnectionTrace(tmp_path / 'trace', max_pending=2)
    entered, release = threading.Event(), threading.Event()
    append = trace._append

    def blocked_write(row):
        entered.set()
        assert release.wait(5)
        append(row)

    monkeypatch.setattr(trace, '_append', blocked_write)
    try:
        assert trace.record('rx', 'in progress')
        assert entered.wait(2)
        assert trace.record('rx', 'queued 1')
        assert trace.record('rx', 'queued 2')
        started = time.monotonic()
        assert not any(trace.record('rx', 'dropped') for _ in range(100))
        future = trace.export(tmp_path / 'snapshot.jsonl')
        assert len(trace._pending) == 3  # Two records and one reserved export.
        assert not future.done()
        trace.close(wait=False)
        assert time.monotonic() - started < .5
        assert not trace.record('rx', 'closed')
    finally:
        release.set()
        trace.close(wait=True)
    history = rows(future.result(timeout=2))
    assert any(row['event'] == 'trace-overflow' and '100 ' in row['detail'] for row in history)
    assert history[-1]['detail'] == 'queued 2'
    assert not trace._thread.is_alive()


def test_disk_failure_is_reported_by_export_without_raising_on_control_writer(tmp_path):
    folder = tmp_path / 'not-a-directory'
    folder.write_text('occupied')
    trace = ConnectionTrace(folder)
    try:
        assert trace.record('diagnostic', 'not writable')
        with pytest.raises(OSError, match='진단 기록 저장 실패'):
            trace.export(tmp_path / 'failed.jsonl').result(timeout=2)
        assert not (tmp_path / 'failed.jsonl').exists()
    finally:
        trace.close(wait=True)


def test_export_cannot_replace_active_trace_file(tmp_path):
    trace = ConnectionTrace(tmp_path)
    try:
        trace.record('rx', 'preserve me')
        with pytest.raises(ValueError, match='덮어쓸'):
            trace.export(tmp_path / 'current.jsonl').result(timeout=2)
        assert rows(tmp_path / 'current.jsonl')[0]['detail'] == 'preserve me'
    finally:
        trace.close(wait=True)


def test_connection_export_signals_after_disconnect_and_nonblocking_unused_close(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    trace = ConnectionTrace(tmp_path / 'trace')
    monkeypatch.setattr(connection, 'ConnectionTrace', lambda: trace)
    owner = connection.Connection()
    assert trace._thread is None
    exported, failures, busy = [], [], []
    owner.diagnostics_exported.connect(exported.append)
    owner.diagnostics_export_failed.connect(failures.append)
    owner.diagnostics_exporting.connect(busy.append)
    owner.trace('연결 해제 후에도 기록 보존')
    assert owner.export_diagnostics(tmp_path / 'offline.jsonl')
    deadline = time.monotonic() + 2
    try:
        while not exported and not failures:
            assert time.monotonic() < deadline
            app.processEvents()
            time.sleep(.005)
        assert not failures and busy == [True, False]
        assert any(row['detail'] == '연결 해제 후에도 기록 보존' for row in rows(tmp_path / 'offline.jsonl'))
    finally:
        owner.close_trace()
        trace.close(wait=True)
    unused = ConnectionTrace(tmp_path / 'unused')
    unused.close(wait=False)
    assert unused._thread is None and not unused.folder.exists()


def test_window_destruction_during_export_does_not_break_writer_shutdown(tmp_path, monkeypatch, caplog):
    from shiboken6 import delete
    app = QApplication.instance() or QApplication([])
    trace = ConnectionTrace(tmp_path / 'trace')
    entered, release = threading.Event(), threading.Event()
    append = trace._append

    def blocked_write(row):
        entered.set()
        assert release.wait(5)
        append(row)

    monkeypatch.setattr(trace, '_append', blocked_write)
    monkeypatch.setattr(connection, 'ConnectionTrace', lambda: trace)
    owner = connection.Connection()
    try:
        owner.trace('pending export')
        assert entered.wait(2)
        owner.export_diagnostics(tmp_path / 'after-close.jsonl')
        owner.close_trace()
        delete(owner)
    finally:
        release.set()
        trace.close(wait=True)
    assert not trace._thread.is_alive()
    assert rows(tmp_path / 'after-close.jsonl')
    assert 'exception calling callback' not in caplog.text
