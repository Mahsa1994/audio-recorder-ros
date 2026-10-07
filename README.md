# audio_recorder_ROS

ROS 2 (Jazzy) audio modality for the FRQNT HRC comfort study. Captures the
Warm Box wireless lavalier microphone and publishes the unprocessed audio so it
lands in the same `ros2 bag` as the robot, physio and vision topics. Operating
steps for sessions live in `../data_collection_steps.txt`, step 5.

The Warm Box lavalier is not a Bluetooth device as far as Linux is concerned:
its USB-C receiver dongle is a standard USB audio device
(`Jieli Technology USB Composite Device`, USB id `4c4a:4155`), showing up as
`USB Composite Device: Audio (hw:N,0)`.

## Pure audio

The published samples are exactly what the receiver delivers: **48000 Hz,
16-bit (S16LE), mono**, the receiver's only format.

- No resampling, filtering, gain or compression is applied anywhere in the node.
- The ALSA hardware device is opened directly at its native format, bypassing
  PipeWire and ALSA's conversion plugins. An unsupported setting therefore fails
  to open rather than being silently converted.
- The receiver's own **Auto Gain Control is switched off** at every start, so
  loudness is not levelled out before it reaches the computer.
- The capture gain is left as found: 147 of 147, which the driver reports as
  -0.94 dB. The mixer state actually in effect is recorded in every bag on
  `/audio/config`.

`bag_to_wav` writes the same samples back out byte for byte. This was verified
on a 30 s live recording.

Bag cost, measured: **~6.0 MB/min, ~30 MB per 5-min session**.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/audio/audio_stamped` | `audio_common_msgs/AudioDataStamped` | 50 msgs/s, 960 samples (20 ms) each, raw S16LE mono PCM (no WAV header). `header.stamp` = capture time of the first sample. |
| `/audio/audio_info` | `audio_common_msgs/AudioInfo` | 48000 Hz, 1 ch, `S16LE`. Latched (transient local). |
| `/audio/config` | `std_msgs/String` | JSON: device, receiver mixer state (AGC, gain), format. Latched; republished whenever the device is (re)opened. |

## Setup and running

See `data_collection_steps.txt` in data collection folder (step 5). In short:

```bash
# one-time, host
sudo apt install ros-jazzy-audio-common-msgs ros-jazzy-rmw-cyclonedds-cpp
pip install --user sounddevice          # already installed on `lair`; needs libportaudio2
cd audio_recorder_ROS && colcon build --symlink-install
# one-time, recorder container: audio_common_msgs must exist where `ros2 bag record` runs (step 6)

# every session
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash && source audio_recorder_ROS/install/setup.bash
ros2 launch audio_recorder audio_recorder.launch.py
ros2 topic hz /audio/audio_stamped      # expect 50.0 Hz
```

## Getting audio out of a bag

```bash
ros2 run audio_recorder bag_to_wav session_P01_T1            # -> session_P01_T1_audio.wav + .json
ros2 run audio_recorder bag_to_wav session_P01_T1 -o p01.wav
```

The JSON sidecar maps the WAV onto the bag's ROS time. WAV sample `i` was
captured at `first_sample_stamp_ns + i * 1e9 / effective_sample_rate`.

If the receiver drops out mid-session, the tool fills the gap with digital
silence so the timeline stays continuous. Each gap is listed in the JSON.
Nothing else is added or changed.

To work directly from the bag instead, use each message's `header.stamp`, not
the bag receive time. The receive time is about 20 ms later because a chunk is
only sent once it is complete.

## Parameters

All of these are node parameters; the launch file exposes the first three.

| Parameter | Default | |
|---|---|---|
| `device` | `USB Composite Device` | Case-insensitive substring of the input device name. Not tied to the card number, which can change between boots. |
| `auto_gain_control` | `false` | Receiver AGC: `false` / `true` / `keep` (leave as is). |
| `capture_volume` | `-1` | Receiver mic gain 0-147 (147 = -0.94 dB, 0 = -28.4 dB); `-1` leaves it. |
| `sample_rate` | `48000` | Must be a rate the device supports natively (the receiver: 48000 only). |
| `chunk_ms` | `20` | Audio per message. |
| `frame_id` | `lavalier_mic` | `header.frame_id`. |
| `status_period` | `10.0` | Seconds between status lines. |
| `silence_warning` | `5.0` | Seconds of silence before warning. |

## Design notes

- **Timestamps** use a sample-count clock locked to the ROS clock. Each chunk's
  stamp is the previous stamp plus its duration, so consecutive messages tile
  time exactly with no scheduling jitter. Every chunk nudges that clock 1% of
  the way toward the measured arrival time, which tracks drift between the
  receiver's crystal and the system clock. A jump larger than 50 ms (lost
  samples) resyncs immediately and is reported. Measured on this receiver: callback
  jitter about 0.05 ms, clock drift -0.4 ppm (0.2 ms per 10 min). The node and the
  recorder run on the same computer (the container shares the host's clock), so
  the stamps and the other topics share one time base.
- **Exclusive access.** Because the node opens the hardware directly, no other
  program (browser, GNOME sound settings level meter) can use the lavalier while
  it runs, and the reverse. If the mic is busy at startup, the node keeps
  retrying every 2 s.
- **Message type.** `audio_common_msgs` is the standard ROS audio message
  package and is released for Jazzy as an apt package, so nothing custom has to
  be built inside the recording container.

## Tests

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
cd src/audio_recorder
python3 -m pytest -p no:cov test     # timestamp-clock unit tests + flake8 + pep257
```

(`-p no:cov` works around the system `pytest-cov` plugin, which is broken on
`lair` because `coverage` isn't installed.)
