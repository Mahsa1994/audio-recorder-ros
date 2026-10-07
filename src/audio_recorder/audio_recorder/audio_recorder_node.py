"""
Capture a microphone and publish timestamped PCM audio for rosbag2.

The samples are published exactly as the device delivers them: no resampling,
filtering, gain or compression.

Publishes (relative names, so a namespace or remap moves all three):
  audio/audio_stamped  audio_common_msgs/AudioDataStamped  raw S16LE mono PCM chunks;
                       header.stamp = capture time of the chunk's first sample
  audio/audio_info     audio_common_msgs/AudioInfo         stream format (latched)
  audio/config         std_msgs/String                     JSON capture settings (latched)
"""

import array
from functools import partial
import json
import math
import queue
import re
import subprocess
import threading
import time

from audio_common_msgs.msg import AudioDataStamped, AudioInfo
from audio_recorder.timing import SampleClock
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
import sounddevice as sd
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
STALL_TIMEOUT = 2.0  # seconds without audio before the stream is reopened
RETRY_PERIOD = 2.0  # seconds between attempts to open a missing or busy device
SILENCE_PEAK = 4  # chunks peaking below this (about -78 dBFS) count as silent
# Mixer controls exposed by the wireless lavalier's USB receiver.
AGC_CONTROL = 'Auto Gain Control'
VOLUME_CONTROL = 'Mic Capture Volume'


def dbfs(value):
    return 20.0 * math.log10(max(value, 1.0) / 32768.0)


class AudioRecorderNode(Node):

    def __init__(self):
        super().__init__('audio_recorder_node')
        self.device = self.declare_parameter('device', 'USB Composite Device').value
        # Must be a rate the device supports natively: the hardware is opened directly,
        # so an unsupported rate fails to open instead of being silently converted.
        self.sample_rate = self.declare_parameter('sample_rate', 48000).value
        self.chunk_ms = self.declare_parameter('chunk_ms', 20).value
        self.frame_id = self.declare_parameter('frame_id', 'lavalier_mic').value
        self.auto_gain_control = self.declare_parameter(
            'auto_gain_control', False, ParameterDescriptor(
                dynamic_typing=True,
                description='true/false switches the receiver AGC on/off, "keep" leaves it'),
        ).value
        self.capture_volume = self.declare_parameter('capture_volume', -1).value
        status_period = self.declare_parameter('status_period', 10.0).value
        self.silence_warning = self.declare_parameter('silence_warning', 5.0).value

        if not isinstance(self.auto_gain_control, bool) and self.auto_gain_control != 'keep':
            raise ValueError('auto_gain_control must be true, false or "keep", '
                             f'got {self.auto_gain_control!r}')
        self.blocksize = max(1, round(self.sample_rate * self.chunk_ms / 1000))

        self.audio_pub = self.create_publisher(AudioDataStamped, 'audio/audio_stamped', 100)
        self.info_pub = self.create_publisher(AudioInfo, 'audio/audio_info', LATCHED)
        self.config_pub = self.create_publisher(String, 'audio/config', LATCHED)
        self.info_pub.publish(AudioInfo(
            channels=1, sample_rate=self.sample_rate, sample_format='S16LE',
            bitrate=self.sample_rate * 16, coding_format='wave'))

        self._stream = None
        self._generation = 0
        self._last_callback = 0.0
        self._next_attempt = 0.0
        self._queue = queue.SimpleQueue()
        self._stats_lock = threading.Lock()
        self._stats = self._new_stats()
        self._silent_for = 0.0
        self._silence_warned = False
        self._running = True
        self._worker = threading.Thread(target=self._process_loop, daemon=True)
        self._worker.start()
        self.create_timer(0.5, self._supervise)
        self.create_timer(status_period, self._report_status)
        self._supervise()

    # -- device handling (executor thread) ----------------------------------

    def _supervise(self):
        now = time.monotonic()
        if self._stream is not None:
            if self._stream.active and now - self._last_callback < STALL_TIMEOUT:
                return
            self.get_logger().error(
                'Audio stream stopped delivering samples (receiver unplugged?) - reopening.')
            self._close_stream()
            self._next_attempt = now
        if now >= self._next_attempt:
            self._next_attempt = now + RETRY_PERIOD
            self._open_stream()

    def _open_stream(self):
        # PortAudio enumerates devices only when initialised, and a re-plugged
        # receiver can come back under a different ALSA card number.
        sd._terminate()
        sd._initialize()
        index = self._find_device()
        if index is None:
            return
        name = sd.query_devices(index)['name']
        card = re.search(r'\(hw:(\d+),\d+\)', name)
        mixer = self._configure_mixer(card.group(1)) if card else {}

        self._generation += 1
        stream = None
        try:
            stream = sd.InputStream(
                device=index, samplerate=self.sample_rate, channels=1, dtype='int16',
                blocksize=self.blocksize, latency='low',
                callback=partial(self._on_audio, self._generation))
            stream.start()
        except sd.PortAudioError as e:
            if stream is not None:
                stream.close(ignore_errors=True)
            self.get_logger().error(
                f'Could not open "{name}": {e}. Is another program (browser, sound settings) '
                'using the microphone? Retrying.', throttle_duration_sec=30)
            return
        self._stream = stream
        self._last_callback = time.monotonic()

        self.get_logger().info(
            f'Recording "{name}": {self.sample_rate} Hz mono S16LE, unprocessed, in '
            f'{self.blocksize * 1000 // self.sample_rate} ms chunks on '
            f'{self.audio_pub.topic_name}. Receiver mixer: {mixer or "n/a"}')
        config = {
            'device': name,
            'mixer': mixer,
            'sample_rate': self.sample_rate,
            'channels': 1,
            'sample_format': 'S16LE',
            'samples_per_message': self.blocksize,
            'processing': 'none: samples exactly as delivered by the device',
            'stamp': 'header.stamp is the capture time (ROS clock) of the first sample',
        }
        self.config_pub.publish(String(data=json.dumps(config)))

    def _find_device(self):
        inputs = [(i, d['name']) for i, d in enumerate(sd.query_devices())
                  if d['max_input_channels'] > 0]
        for index, name in inputs:
            if self.device.lower() in name.lower():
                return index
        listing = '; '.join(f'"{name}"' for _, name in inputs)
        # PortAudio also hides a device another program has open, so busy looks like missing.
        self.get_logger().error(
            f'No input device matching "{self.device}". Is the receiver plugged in, and not '
            'held by another program (browser, sound settings, arecord)? Retrying. '
            f'Available inputs: {listing}', throttle_duration_sec=30)
        return None

    def _configure_mixer(self, card):
        """Apply the requested receiver settings and return what the mixer reports."""
        if isinstance(self.auto_gain_control, bool):
            self._amixer(card, 'cset', AGC_CONTROL, 'on' if self.auto_gain_control else 'off')
        if self.capture_volume >= 0:
            self._amixer(card, 'cset', VOLUME_CONTROL, str(self.capture_volume))
        state = {}
        for control in (AGC_CONTROL, VOLUME_CONTROL):
            match = re.search(r'^\s*: values=(.*)$', self._amixer(card, 'cget', control) or '',
                              re.MULTILINE)
            if match:
                state[control] = match.group(1)
        return state

    def _amixer(self, card, command, control, *value):
        try:
            result = subprocess.run(
                ['amixer', '-c', card, command, f'name={control}', *value],
                capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().warning(f'amixer failed: {e}')
            return None
        if result.returncode != 0:
            # Other microphones may lack these controls; only complain about requested changes.
            if command == 'cset':
                self.get_logger().warning(
                    f'Could not set "{control}" on card {card}: {result.stderr.strip()}')
            return None
        return result.stdout

    def _close_stream(self):
        if self._stream is not None:
            self._stream.close(ignore_errors=True)
            self._stream = None

    # -- audio path ----------------------------------------------------------

    def _on_audio(self, generation, indata, frames, time_info, status):
        # PortAudio's thread: take the arrival time first, hand off, return quickly.
        arrival_ns = self.get_clock().now().nanoseconds
        self._last_callback = time.monotonic()
        self._queue.put((generation, arrival_ns, indata[:, 0].copy(), status.input_overflow))

    def _process_loop(self):
        generation = None
        while self._running:
            try:
                item_generation, arrival_ns, samples, overflow = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item_generation != generation:  # new stream: restart the timeline
                generation = item_generation
                clock = SampleClock(self.sample_rate)
            first_ns, resynced = clock.stamp(arrival_ns, len(samples), force_resync=overflow)
            msg = AudioDataStamped()
            msg.header.stamp = Time(nanoseconds=first_ns).to_msg()
            msg.header.frame_id = self.frame_id
            msg.audio.data = array.array('B', samples.astype('<i2').tobytes())
            self.audio_pub.publish(msg)
            self._update_stats(samples, overflow, resynced)

    # -- health reporting ----------------------------------------------------

    @staticmethod
    def _new_stats():
        return {'seconds': 0.0, 'sum_squares': 0.0, 'peak': 0, 'clipped': 0,
                'overflows': 0, 'resyncs': 0}

    def _update_stats(self, samples, overflow, resynced):
        peak = int(np.abs(samples.astype(np.int32)).max())
        duration = len(samples) / self.sample_rate
        with self._stats_lock:
            stats = self._stats
            stats['seconds'] += duration
            stats['sum_squares'] += float(np.square(samples, dtype=np.float64).sum())
            stats['peak'] = max(stats['peak'], peak)
            stats['clipped'] += int(np.count_nonzero((samples == 32767) | (samples == -32768)))
            stats['overflows'] += int(overflow)
            stats['resyncs'] += int(resynced)

        if peak >= SILENCE_PEAK:
            if self._silence_warned:
                self.get_logger().info('Microphone signal is back.')
            self._silent_for, self._silence_warned = 0.0, False
            return
        self._silent_for += duration
        if self._silent_for >= self.silence_warning and not self._silence_warned:
            self._silence_warned = True
            self.get_logger().warning(
                f'Microphone silent for {self._silent_for:.0f} s - is the lavalier transmitter '
                'switched on and linked to the receiver?')

    def _report_status(self):
        with self._stats_lock:
            stats, self._stats = self._stats, self._new_stats()
        if self._stream is None or stats['seconds'] == 0:
            return  # open and stall problems are reported where they happen
        rms = math.sqrt(stats['sum_squares'] / (stats['seconds'] * self.sample_rate))
        line = (f'{stats["seconds"]:.1f} s captured | level RMS {dbfs(rms):.1f} dBFS, '
                f'peak {dbfs(stats["peak"]):.1f} dBFS')
        problems = []
        if stats['clipped']:
            problems.append(f'{stats["clipped"]} clipped samples - lower capture_volume')
        if stats['overflows']:
            problems.append(f'{stats["overflows"]} input overflows (audio lost)')
        if stats['resyncs']:
            problems.append(f'{stats["resyncs"]} timestamp resyncs')
        if problems:
            self.get_logger().warning(f'{line} | {"; ".join(problems)}')
        else:
            self.get_logger().info(line)

    def destroy_node(self):
        self._running = False
        self._close_stream()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AudioRecorderNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
