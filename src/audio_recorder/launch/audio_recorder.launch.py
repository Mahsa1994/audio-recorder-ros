from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    ('device', 'USB Composite Device',
     'Substring of the input device name (the lavalier USB receiver shows up as '
     '"USB Composite Device: Audio (hw:N,0)").'),
    ('auto_gain_control', 'false',
     'Receiver AGC: false = unprocessed signal (default), true = receiver levels the '
     'loudness, keep = leave the current setting.'),
    ('capture_volume', '-1',
     'Receiver mic gain, 0-147 (147 = -0.94 dB). -1 leaves the current setting.'),
]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in ARGUMENTS],
        Node(
            package='audio_recorder',
            executable='audio_recorder_node',
            name='audio_recorder_node',
            output='screen',
            parameters=[{name: LaunchConfiguration(name) for name, _, _ in ARGUMENTS}],
        ),
    ])
