#!/usr/bin/env python3

import actionlib
import rospy
import subprocess
import time

from audio_recorder_msgs.msg import StartRecordingAction, StartRecordingFeedback, StartRecordingGoal, StartRecordingResult
from audio_recorder_msgs.msg import StopRecordingAction, StopRecordingFeedback, StopRecordingGoal, StopRecordingResult
from audio_recorder_msgs.msg import Status
from std_msgs.msg import Bool

def defaultFilename():
    return time.strftime("%Y%m%d%H%M%S")+".wav"

class AudioRecorderNode:
    def __init__(self, output_dir, card_id=0, device_id=0, bitrate=44100, channels=1):
        self.hw_id = 'hw:{0},{1}'.format(card_id, device_id)
        self.bitrate = bitrate
        self.output_dir = output_dir
        self.channels = channels

        self.start_recording_srv = actionlib.SimpleActionServer('start_recording', StartRecordingAction, self.startRecording_actionHandler, False)
        self.stop_recording_srv = actionlib.SimpleActionServer('stop_recording', StopRecordingAction, self.stopRecording_actionHandler, False)

    def run(self):
        self.start_recording_srv.start()
        self.stop_recording_srv.start()

        self.is_recording = False
        self.status_pub = rospy.Publisher('recorder_status', Status, queue_size=1)
        self.is_recording_pub = rospy.Publisher('is_recording', Bool, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

            status = Status()
            rec = Bool()
            rec.data = self.is_recording
            if self.is_recording:
                status.status = Status.RECORDING | Status.RUNNING
            else:
                status.status = Status.RUNNING
            self.status_pub.publish(status)
            self.is_recording_pub.publish(rec)

    def startRecording_actionHandler(self, req):
        if self.is_recording:
            rospy.logwarn("Unable to start new audio recording; the previous one is still in progress")
            result = StartRecordingResult()
            result.success = False
            self.start_recording_srv.set_aborted(result, "Previous recording is still in progress")
            return

        self.is_recording = True
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

        if req.duration != 0:
            rate = rospy.Rate(1)
            for i in range(req.duration):
                rate.sleep()
                feedback = StartRecordingFeedback()
                feedback.time_elapsed = i+1
                feedback.time_remaining = req.duration - i - 1
                self.start_recording_srv.publish_feedback(feedback)

            self.is_recording = False
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
            self.stop_recording_srv.set_aborted(result, "No recording to stop")
            return

        self.alsa_proc.terminate()
        now = rospy.get_rostime()
        elapsed = now - self.record_start_time

        result = StopRecordingResult()
        result.success = True
        result.path = self.wav_path
        result.duration = int(elapsed.to_sec())

        self.is_recording = False
        self.stop_recording_srv.set_succeeded(result)
