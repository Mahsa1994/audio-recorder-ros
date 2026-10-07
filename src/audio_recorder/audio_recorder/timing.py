"""Capture timestamps for streamed audio (no ROS dependencies)."""


class SampleClock:
    """
    Assigns system-clock capture times to consecutive chunks of audio.

    Each chunk's first-sample time is predicted from the previous chunk plus
    its sample count, so timestamps are free of scheduling jitter and the
    chunks tile the timeline exactly. Every chunk nudges the prediction a small
    fraction (`gain`) of the way toward the measured arrival time, which keeps
    following any drift between the sound card's sample clock and the system
    clock. A disagreement larger than `resync_threshold` seconds (dropped
    samples, a stalled device) snaps straight to the measurement instead.
    """

    def __init__(self, sample_rate, gain=0.01, resync_threshold=0.05):
        self.sample_rate = float(sample_rate)
        self.gain = gain
        self.resync_threshold = resync_threshold
        self._epoch_ns = None  # times are kept relative to this for float precision
        self._next = None  # predicted time of the next chunk's first sample

    def stamp(self, arrival_ns, frames, force_resync=False):
        """
        Timestamp a chunk of `frames` samples delivered at `arrival_ns`.

        Returns (first_sample_ns, resynced), where `resynced` is True when the
        timeline had to jump to the measured time (never for the first chunk).
        """
        if self._epoch_ns is None:
            self._epoch_ns = arrival_ns
        duration = frames / self.sample_rate
        measured = (arrival_ns - self._epoch_ns) * 1e-9 - duration
        resynced = False
        if self._next is None or force_resync or \
                abs(measured - self._next) > self.resync_threshold:
            resynced = self._next is not None
            start = measured
        else:
            start = self._next + self.gain * (measured - self._next)
        self._next = start + duration
        return self._epoch_ns + round(start * 1e9), resynced
