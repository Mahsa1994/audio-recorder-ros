from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'audio_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mahsa Hasheminejad',
    maintainer_email='ama.hasheminejad@gmail.com',
    description='Captures a microphone and publishes timestamped PCM audio for rosbag2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_recorder_node = audio_recorder.audio_recorder_node:main',
            'bag_to_wav = audio_recorder.bag_to_wav:main',
        ],
    },
)
