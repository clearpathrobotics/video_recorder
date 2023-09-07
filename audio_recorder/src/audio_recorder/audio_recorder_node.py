#!/usr/bin/env python3

import actionlib
import datetime
import json
import os
import rospy
import subprocess
import tf2_ros
import time

from audio_recorder_msgs.msg import StartRecordingAction, StartRecordingFeedback, StartRecordingGoal, StartRecordingResult
from audio_recorder_msgs.msg import StopRecordingAction, StopRecordingFeedback, StopRecordingGoal, StopRecordingResult
from audio_recorder_msgs.msg import Status
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool

from tf.transformations import euler_from_quaternion

def defaultFilename():
    return time.strftime("%Y-%m-%d_%H-%M-%S")+".wav"

class AudioRecorderNode:
    def __init__(self, output_dir, card_id=0, device_id=0, bitrate=44100, channels=1, record_metadata=False, mic_frame="mic_frame"):
        self.hw_id = 'hw:{0},{1}'.format(card_id, device_id)
        self.bitrate = bitrate
        self.output_dir = output_dir
        self.channels = channels
        self.record_metadata = record_metadata
        self.mic_frame = mic_frame

        self.createStorageDirectory()

        self.start_recording_srv = actionlib.SimpleActionServer('start_recording', StartRecordingAction, self.startRecording_actionHandler, False)
        self.stop_recording_srv = actionlib.SimpleActionServer('stop_recording', StopRecordingAction, self.stopRecording_actionHandler, False)

    def createStorageDirectory(self):
        try:
            os.makedirs(self.output_dir)
            rospy.logwarn("Output directory {0} created".format(self.output_dir))
        except FileExistsError:
            rospy.loginfo("Output directory already exists")

    def run(self):
        self.start_recording_srv.start()
        self.stop_recording_srv.start()

        self.status_pub = rospy.Publisher('recorder_status', Status, queue_size=1)
        self.is_recording_pub = rospy.Publisher('is_recording', Bool, queue_size=1, latch=True)

        # publish the initial state of the is_recording topic
        self.is_recording = False
        self.notify_is_recording_changed()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

            status = Status()
            if self.is_recording:
                status.status = Status.RECORDING | Status.RUNNING
            else:
                status.status = Status.RUNNING
            self.status_pub.publish(status)

    def notify_is_recording_changed(self):
        """Publish to the latched .../is_recording topic
        """
        self.is_recording_pub.publish(Bool(self.is_recording))

    def startRecording_actionHandler(self, req):
        if self.is_recording:
            rospy.logwarn("Unable to start new audio recording; the previous one is still in progress")
            result = StartRecordingResult()
            result.success = False
            result.path = self.wav_path
            self.start_recording_srv.set_succeeded(result)
            return

        self.is_recording = True
        self.notify_is_recording_changed()
        if req.filename:
            self.wav_path = "{0}/{1}".format(self.output_dir, req.filename)
        else:
            self.wav_path = "{0}/{1}".format(self.output_dir, defaultFilename())

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

        self.record_start_time = rospy.get_rostime()
        self.alsa_proc = subprocess.Popen(cmd)

        if self.record_metadata:
            self.saveMetaData()

        if req.duration != 0:
            rate = rospy.Rate(1)
            for i in range(req.duration):
                rate.sleep()
                feedback = StartRecordingFeedback()
                feedback.time_elapsed = i+1
                feedback.time_remaining = req.duration - i - 1
                self.start_recording_srv.publish_feedback(feedback)

            self.is_recording = False
            self.notify_is_recording_changed()
            self.alsa_proc.communicate()

        result = StartRecordingResult()
        result.success = True
        result.path = self.wav_path
        self.start_recording_srv.set_succeeded(result)


    def stopRecording_actionHandler(self, req):
        if not self.is_recording:
            rospy.logwarn("Unable to stop recording; no recording in progress")
            result = StopRecordingResult()
            result.success = False
            self.stop_recording_srv.set_succeeded(result)
            return

        self.alsa_proc.terminate()
        now = rospy.get_rostime()
        elapsed = now - self.record_start_time

        result = StopRecordingResult()
        result.success = True
        result.path = self.wav_path
        result.duration = int(elapsed.to_sec())

        self.is_recording = False
        self.notify_is_recording_changed()
        self.stop_recording_srv.set_succeeded(result)

    def saveMetaData(self):
        json_path = f"{self.wav_path}.json"
        robot_pose = self.lookupTransform("map", "base_link")
        mic_pose = self.lookupTransform("base_link", self.mic_frame)

        json_data = {
            "time": str(datetime.datetime.now()),
            "file": self.wav_path,

            "alsa_device": self.hw_id,
            "bitrate": self.bitrate,
            "channels": self.channels,

            "robot_pose": {
                "linear": {
                    "x": robot_pose.linear.x,
                    "y": robot_pose.linear.y,
                    "z": robot_pose.linear.z
                },
                "angular": {
                    "x": robot_pose.angular.x,
                    "y": robot_pose.angular.y,
                    "z": robot_pose.angular.z
                }
            },

            "mic_pose": {
                "linear": {
                    "x": mic_pose.linear.x,
                    "y": mic_pose.linear.y,
                    "z": mic_pose.linear.z
                },
                "angular": {
                    "x": mic_pose.angular.x,
                    "y": mic_pose.angular.y,
                    "z": mic_pose.angular.z
                }
            }
        }

        fout = open(json_path, 'w')
        json.dump(json_data, fout, indent=2)
        fout.close()

    def lookupTransform(self, fixed_frame, target_frame):
        try:
            tf_buf = tf2_ros.Buffer(rospy.Duration(2.0))
            tf_listener = tf2_ros.TransformListener(tf_buf)
            tf_stamped = tf_buf.lookup_transform(fixed_frame, target_frame, rospy.Time(0), rospy.Duration(5.0))

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
            rospy.logwarn(f"Failed to lookup transform from {fixed_frame} to {target_frame}: {err}")
            return Twist(
                Vector3(0, 0, 0),
                Vector3(0, 0, 0)
            )
