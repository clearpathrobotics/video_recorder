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
import datetime
import json
import os
import subprocess
import time

from audio_recorder_msgs.action import (
    StartRecording,
    StopRecording,
)
from audio_recorder_msgs.msg import Status
from geometry_msgs.msg import Twist, Vector3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
import tf2_ros
from tf_transformations import euler_from_quaternion

def defaultFilename():
    return time.strftime('%Y-%m-%d_%H-%M-%S')+'.wav'

class AudioRecorderNode(Node):
    def __init__(self):
        super().__init__('audio_recorder_node')

        self.card_id = self.get_parameter_or('card_id', 0)
        self.device_id = self.get_parameter_or('device_id', 0)
        self.bitrate = self.get_parameter_or('bitrate', 44100)
        self.output_dir = self.get_parameter_or('out_dir', '/tmp')
        self.mount_dir = self.get_parameter_or('mount_dir', '')
        self.channels = self.get_parameter_or('channels', 1)
        self.record_metadata = self.get_parameter_or('record_metadata', True)
        self.mic_frame = self.get_parameter_or('mic_frame', 'mic_frame')

        self.hw_id = f'hw:{self.card_id},{self.device_id}'

        self.createStorageDirectory()

        self.start_recording_srv = ActionServer(
            self,
            StartRecording,
            'start_recording',
            self.startRecording_actionHandler,
        )
        self.stop_recording_srv = ActionServer(
            self,
            StopRecording,
            'stop_recording',
            self.stopRecording_actionHandler,
        )

        self.status_pub = self.create_publisher(Status, 'recorder_status', qos_profile_sensor_data)
        self.is_recording_pub = self.create_publisher(Bool, 'is_recording', qos_profile_sensor_data)

        # publish the initial state of the is_recording topic
        self.is_recording = False
        self.notify_is_recording_changed()

        self.pub_timer = self.create_timer(0.1, self.on_publish_timer)

    def createStorageDirectory(self):
        try:
            os.makedirs(self.output_dir)
            self.get_logger().warning(f'Output directory {self.output_dir} created')
        except FileExistsError:
            self.get_logger().info('Output directory already exists')

    def on_publish_timer(self):
        status = Status()
        if self.is_recording:
            status.status = Status.RECORDING | Status.RUNNING
        else:
            status.status = Status.RUNNING
        self.status_pub.publish(status)

    def notify_is_recording_changed(self):
        """Publish to the latched .../is_recording topic"""
        b = Bool()
        b.data = self.is_recording
        self.is_recording_pub.publish(b)

    def startRecording_actionHandler(self, req):
        if self.is_recording:
            self.get_logger().warning('Unable to start new audio recording; the previous one is still in progress')
            result = StartRecording.Result()
            result.success = False
            result.path = self.result_path
            result.header.stamp = self.get_clock().now()
            result.header.frame_id = self.camera_frame
            self.start_recording_srv.set_succeeded(result)
            return

        self.is_recording = True
        self.notify_is_recording_changed()
        if req.filename:
            filename = req.filename
        else:
            filename = defaultFilename()
        self.wav_path = f"{self.output_dir}/{filename}"
        if self.mount_dir:
            self.result_path = f"{self.mount_dir}/{filename}"
        else:
            self.result_path = self.wav_path

        self.get_logger().warning(self.result_path)

        if req.duration == 0:
            cmd = ['arecord',
                   '-D', self.hw_id,
                   '-r', str(self.bitrate),
                   '-c', str(self.channels),
                   self.wav_path]
        else:
            cmd = ['arecord',
                   '-D', self.hw_id,
                   '-r', str(self.bitrate),
                   '-c', str(self.channels),
                   '-d', str(req.duration),
                   self.wav_path]

        self.record_start_time = self.get_clock().now()
        self.alsa_proc = subprocess.Popen(cmd)

        if self.record_metadata:
            self.saveMetaData()

        if req.duration != 0:
            rate = self.create_rate(1)
            for i in range(req.duration):
                rate.sleep()
                feedback = StartRecording.Feedback()
                feedback.time_elapsed = i+1
                feedback.time_remaining = req.duration - i - 1
                self.start_recording_srv.publish_feedback(feedback)

            self.is_recording = False
            self.notify_is_recording_changed()
            self.alsa_proc.communicate()

        result = StartRecording.Result()
        result.success = True
        result.path = self.result_path
        result.header.stamp = self.get_clock().now()
        result.header.frame_id = self.mic_frame
        self.start_recording_srv.set_succeeded(result)


    def stopRecording_actionHandler(self, req):
        if not self.is_recording:
            self.get_logger().warning('Unable to stop recording; no recording in progress')
            result = StopRecording.Result()
            result.success = False
            result.header.stamp = self.get_clock().now()
            result.header.frame_id = self.mic_frame
            self.stop_recording_srv.set_succeeded(result)
            return

        self.alsa_proc.terminate()
        now = self.get_clock().now()
        elapsed = now - self.record_start_time

        result = StopRecording.Result()
        result.success = True
        result.path = self.result_path
        result.duration = int(elapsed.to_sec())
        result.header.stamp = self.get_clock().now()
        result.header.frame_id = self.mic_frame

        self.is_recording = False
        self.notify_is_recording_changed()
        self.stop_recording_srv.set_succeeded(result)

    def saveMetaData(self):
        json_path = f'{self.wav_path}.json'
        robot_pose = self.lookupTransform('map', 'base_link')
        mic_pose = self.lookupTransform('base_link', self.mic_frame)

        json_data = {
            'time': str(datetime.datetime.now()),
            'file': self.wav_path,

            'alsa_device': self.hw_id,
            'bitrate': self.bitrate,
            'channels': self.channels,

            'robot_pose': {
                'linear': {
                    'x': robot_pose.linear.x,
                    'y': robot_pose.linear.y,
                    'z': robot_pose.linear.z
                },
                'angular': {
                    'x': robot_pose.angular.x,
                    'y': robot_pose.angular.y,
                    'z': robot_pose.angular.z
                }
            },

            'mic_pose': {
                'linear': {
                    'x': mic_pose.linear.x,
                    'y': mic_pose.linear.y,
                    'z': mic_pose.linear.z
                },
                'angular': {
                    'x': mic_pose.angular.x,
                    'y': mic_pose.angular.y,
                    'z': mic_pose.angular.z
                }
            }
        }

        fout = open(json_path, 'w')
        json.dump(json_data, fout, indent=2)
        fout.close()

    def lookupTransform(self, fixed_frame, target_frame):
        try:
            tf_buf = tf2_ros.Buffer(rclpy.Duration(2.0))
            _ = tf2_ros.TransformListener(tf_buf)
            tf_stamped = tf_buf.lookup_transform(
                fixed_frame,
                target_frame,
                rclpy.time.Time()
            )

            (roll, pitch, yaw) = euler_from_quaternion([
                tf_stamped.transform.rotation.x,
                tf_stamped.transform.rotation.y,
                tf_stamped.transform.rotation.z,
                tf_stamped.transform.rotation.w,
            ])

            return Twist(
                Vector3(
                    tf_stamped.transform.translation.x,
                    tf_stamped.transform.translation.y,
                    tf_stamped.transform.translation.z
                ),
                Vector3(
                    roll, pitch, yaw
                )
            )

        except Exception as err:
            self.get_logger().warning(f'Failed to lookup transform from {fixed_frame} to {target_frame}: {err}')
            return Twist(
                Vector3(0, 0, 0),
                Vector3(0, 0, 0)
            )


def main():
    rclpy.init()
    node = AudioRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
