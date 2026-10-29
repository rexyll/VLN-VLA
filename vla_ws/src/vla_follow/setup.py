import os
from glob import glob
from setuptools import setup

package_name = 'vla_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yml') + glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'audio'),
         glob('audio/*.wav')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='earl',
    maintainer_email='earl@example.com',
    description='Voice-command person-following demo '
                '(ROS 2 Jazzy + Gazebo Harmonic + Vosk)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'person_walk = vla_follow.person_walk:main',
            'voice_node = vla_follow.voice_node:main',
            'follower = vla_follow.follower:main',
            'voice_trigger = vla_follow.voice_trigger:main',
            'depth_percept = vla_follow.depth_percept:main',
            'person_vision = vla_follow.person_vision:main',
            'gt_tf = vla_follow.gt_tf:main',
            'cmd_mux = vla_follow.cmd_mux:main',
            'gz_viz = vla_follow.gz_viz:main',
            'drink_task = vla_follow.drink_task:main',
        ],
    },
)
