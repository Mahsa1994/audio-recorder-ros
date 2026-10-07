from audio_recorder.timing import SampleClock
import numpy as np
import pytest

FS = 48000


def test_clock_tiles_chunks_and_ignores_jitter():
    clock = SampleClock(FS)
    rng = np.random.default_rng(3)
    start, frames = 1_790_000_000 * 10**9, 960
    stamps = []
    for k in range(3000):
        arrival = start + (k + 1) * 20_000_000 + int(rng.uniform(0, 2e6))  # 0-2 ms late
        stamps.append(clock.stamp(arrival, frames)[0])
    steps = np.diff(stamps[1000:])
    # each step moves at most gain * error = 0.01 * 1 ms away from exactly 20 ms
    assert np.abs(steps - 20_000_000).max() < 15_000
    truth = start + np.arange(1000, 3000) * 20_000_000
    assert np.abs(np.array(stamps[1000:]) - truth).max() < 2_000_000


def test_clock_follows_drift():
    clock = SampleClock(FS)
    start, frames, ppm = 10**18, 960, 300  # device clock 300 ppm slow
    true_period = 20_000_000 * (1 + ppm * 1e-6)
    for k in range(5000):
        arrival = start + round((k + 1) * true_period)
        stamp, resynced = clock.stamp(arrival, frames)
        assert not resynced
    assert abs(stamp - (start + round(k * true_period))) < 1_000_000  # < 1 ms after 100 s


def test_clock_resyncs_after_dropout():
    clock = SampleClock(FS)
    start = 10**18
    for k in range(100):
        clock.stamp(start + (k + 1) * 20_000_000, 960)
    arrival = start + 101 * 20_000_000 + 300_000_000  # 300 ms of audio lost
    stamp, resynced = clock.stamp(arrival, 960)
    assert resynced
    assert stamp == pytest.approx(arrival - 20_000_000, abs=1_000)
