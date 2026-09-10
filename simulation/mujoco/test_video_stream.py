"""Video transport must never present an old frame as a live simulator view."""
import json
import threading
import time
from http.server import ThreadingHTTPServer
from urllib.request import urlopen
from urllib.error import HTTPError

import pytest
from video_stream import VideoFrames, handler_for


@pytest.fixture
def endpoint():
    frames = VideoFrames()
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(frames))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield frames, f'http://127.0.0.1:{server.server_port}'
    server.shutdown()
    server.server_close()
    thread.join()


def test_missing_and_stale_frames_are_unavailable(endpoint):
    frames, url = endpoint
    for stale in (False, True):
        if stale:
            frames.publish(b'jpeg', 10)
            frames.updated = time.monotonic() - 3
        with pytest.raises(HTTPError) as error:
            urlopen(url + '/frame.jpg')
        assert error.value.code == 503
        with urlopen(url + '/status') as response:
            assert json.load(response)['ready'] is False


def test_fresh_frame_and_unknown_path(endpoint):
    frames, url = endpoint
    frames.publish(b'jpeg-bytes', 12.5)
    with urlopen(url + '/frame.jpg') as response:
        assert response.read() == b'jpeg-bytes'
        assert response.headers['Content-Type'] == 'image/jpeg'
        assert response.headers['Cache-Control'] == 'no-store'
        assert response.headers['X-SpotOMG-Video'] == '1'
        assert response.headers['X-Simulation-Time'] == '12.5'
    with pytest.raises(HTTPError) as error:
        urlopen(url + '/anything-else')
    assert error.value.code == 404


def test_waits_for_new_frame_without_repeating_old_frame(endpoint):
    frames, url = endpoint
    frames.publish(b'first', 1)
    with urlopen(url + '/frame.jpg?after=1') as response:
        assert response.status == 204
        assert response.read() == b''
    timer = threading.Timer(.05, lambda: frames.publish(b'next', 2))
    timer.start()
    try:
        with urlopen(url + '/frame.jpg?after=1') as response:
            assert response.read() == b'next'
            assert response.headers['X-Frame-Sequence'] == '2'
    finally:
        timer.join()
