#!/usr/bin/env python3
# Software License Agreement (BSD)
#
# @author    Chris Iverach-Brereton <civerachb@clearpathrobotics.com>
# @copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'out_dir',
        default_value=os.environ['HOME'] or '/tmp',
        description='Output directory for recordings and metadata',
    ),
    DeclareLaunchArgument(
        'mount_dir',
        default_value='',
        description='Optional mount path when running this node in a container',
    ),

    DeclareLaunchArgument(
        'card',
        default_value='0',
        description='ALSA card ID',
    ),
    DeclareLaunchArgument(
        'device',
        default_value='0',
        description='ALSA devce ID',
    ),
    DeclareLaunchArgument(
        'bitrate',
        default_value='44100',
        description='ALSA device bitrate',
    ),
    DeclareLaunchArgument(
        'channels',
        default_value='1',
        description='ALSA device channels (1: mono, 2: stereo)',
    ),
    DeclareLaunchArgument(
        'format',
        default_value='S16_LE',
        description='ALSA recording format (e.g. S16_LE, S8_LE)',
    ),

    DeclareLaunchArgument(
        'record_metadata',
        default_value='true',
        description='Record meta-data about recordings',
    ),
    DeclareLaunchArgument(
        'mic_frame',
        default_value='mic_frame',
        description='URDF frame for the mic',
    ),
]


def generate_launch_description():
    out_dir = LaunchConfiguration('out_dir')
    mount_dir = LaunchConfiguration('mount_dir')

    card = LaunchConfiguration('card')
    device = LaunchConfiguration('device')
    bitrate = LaunchConfiguration('bitrate'),
    channels = LaunchConfiguration('channels')
    format = LaunchConfiguration('format')
    record_metadata = LaunchConfiguration('record_metadata')
    mic_frame = LaunchConfiguration('mic_frame')

    audio_recorder_node = Node(
        package='audio_recorder',
        executable='audio_recorder_node',
        output='screen',
        parameters=[
            {'bitrate': bitrate},
            {'card_id': card},
            {'channels': channels},
            {'device_id': device},
            {'format': format},
            {'mic_frame': mic_frame},
            {'mount_dir': mount_dir},
            {'out_dir': out_dir},
            {'record_metadata': record_metadata},
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(audio_recorder_node)
    return ld
