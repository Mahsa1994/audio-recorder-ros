"""
Export the audio recorded in a rosbag2 bag to a WAV file plus a timing sidecar.

Gaps in the recording (receiver unplugged, lost samples) are filled with
silence, so WAV sample i always maps to ROS time
    first_sample_stamp_ns + i * 1e9 / effective_sample_rate
using the values in the sidecar JSON, which lines the audio up with the other
topics in the bag.
"""

import argparse
import json
from pathlib import Path
import sys
import wave

from audio_common_msgs.msg import AudioDataStamped, AudioInfo
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py


def read_audio(bag, topic, info_topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag)),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    available = {meta.name for meta in reader.get_all_topics_and_types()}
    if topic not in available:
        sys.exit(f'Topic {topic} is not in {bag}. Topics found: {", ".join(sorted(available))}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in (topic, info_topic)
                                                       if t in available]))
    info, chunks = None, []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == info_topic:
            info = deserialize_message(data, AudioInfo)
        else:
            msg = deserialize_message(data, AudioDataStamped)
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            chunks.append((stamp, np.frombuffer(msg.audio.data, dtype='<i2')))
    return info, chunks


def assemble(chunks, sample_rate, gap_threshold):
    """Concatenate chunks in time order, padding gaps with silence."""
    chunks.sort(key=lambda chunk: chunk[0])
    parts, starts, gaps, overlaps = [], [], [], []
    position = 0  # samples written so far
    expected = None  # stamp at which the next chunk should start
    for stamp, samples in chunks:
        if expected is not None:
            missing = round((stamp - expected) * 1e-9 * sample_rate)
            if missing > gap_threshold:
                parts.append(np.zeros(missing, dtype='<i2'))
                gaps.append({'at_s': position / sample_rate, 'duration_s': missing / sample_rate})
                position += missing
            elif missing < -gap_threshold:
                overlaps.append({'at_s': position / sample_rate,
                                 'duration_s': -missing / sample_rate})
        starts.append((position, stamp))
        parts.append(samples)
        position += len(samples)
        expected = stamp + round(len(samples) * 1e9 / sample_rate)
    return np.concatenate(parts), starts, gaps, overlaps


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('bag', type=Path, help='bag directory (or a single .mcap/.db3 file)')
    parser.add_argument('-o', '--output', type=Path,
                        help='output .wav (default: <bag name>_audio.wav in the current dir)')
    parser.add_argument('--topic', default='/audio/audio_stamped')
    parser.add_argument('--info-topic', default='/audio/audio_info')
    parser.add_argument('--sample-rate', type=int,
                        help='use when the bag has no AudioInfo message')
    parser.add_argument('--gap-threshold-ms', type=float, default=10.0,
                        help='timeline jumps longer than this are filled with silence')
    args = parser.parse_args(argv)

    info, chunks = read_audio(args.bag, args.topic, args.info_topic)
    if not chunks:
        sys.exit(f'No messages on {args.topic} in {args.bag}')
    if info is not None:
        if info.channels != 1 or info.sample_format != 'S16LE':
            sys.exit(f'Unsupported stream: {info.channels} channels, {info.sample_format}')
        sample_rate = info.sample_rate
    elif args.sample_rate:
        sample_rate = args.sample_rate
    else:
        sys.exit(f'No AudioInfo on {args.info_topic}; pass --sample-rate')

    audio, starts, gaps, overlaps = assemble(
        chunks, sample_rate, args.gap_threshold_ms * 1e-3 * sample_rate)
    first_stamp = int(starts[0][1])
    if len(starts) > 1:
        # Line through all chunk stamps: averages out per-chunk timing noise and
        # absorbs the sound card's clock drift relative to the ROS clock.
        positions = np.array([position for position, _ in starts], dtype=np.float64)
        seconds = np.array([(stamp - first_stamp) * 1e-9 for _, stamp in starts])
        seconds_per_sample, intercept = np.polyfit(positions, seconds, 1)
        effective_rate = 1.0 / seconds_per_sample
        first_stamp += round(intercept * 1e9)
    else:
        effective_rate = float(sample_rate)

    output = args.output or Path(f'{args.bag.resolve().name}_audio.wav')
    with wave.open(str(output), 'wb') as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(audio.tobytes())
    timing = {
        'wav': output.name,
        'bag': str(args.bag.resolve()),
        'topic': args.topic,
        'sample_rate': sample_rate,
        'num_samples': len(audio),
        'duration_s': len(audio) / sample_rate,
        'first_sample_stamp_ns': first_stamp,
        'effective_sample_rate': effective_rate,
        'messages': len(chunks),
        'gaps_filled_with_silence': gaps,
        'overlaps': overlaps,
    }
    sidecar = output.with_suffix('.json')
    sidecar.write_text(json.dumps(timing, indent=2) + '\n')

    print(f'Wrote {output} ({len(audio) / sample_rate:.2f} s, {sample_rate} Hz) and {sidecar}')
    print(f'  first sample at ROS time {first_stamp / 1e9:.6f} s; '
          f'effective rate {effective_rate:.3f} Hz')
    if gaps:
        total = sum(gap['duration_s'] for gap in gaps)
        print(f'  WARNING: {len(gaps)} gaps ({total:.3f} s total) filled with silence')
    if overlaps:
        print(f'  WARNING: {len(overlaps)} chunks overlapped the previous one (clock jump?)')


if __name__ == '__main__':
    main()
