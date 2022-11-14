video_recorder
=================

This package contains the `video_recorder_node`, which can be used to record a video file from
a topic publishing `sensor_msgs/Image` data and save individual frames.

The node's configuration is done via `rosparams`:
```yaml
video_recorder_node:
  fps: 30.0
  output_height: 480
  output_width: 640
  topic: /camera/image_raw
  out_dir: /home/administrator/capture
  max_duration: 0
  record_metadata: false
  camera_frame: camera
  enable_zoom: false
```
- `topic` is fairly self-explanatory: this is the ROS topic to subscribe to. Default: `/camera/image_raw`
- `fps` is the FPS of the output file. This should be set to the same FPS that the camera node publihes at. Default: `30`
- `output_height` is the frame height of the output video files
- `output_width` is the frame width of the output video files.
- `out_dir` is the directory on-disk to save the recorded videos.  The directory is created if it doesn't already exist
  on the disk. Default: `/tmp`
- `max_duration` is the absolute maximum duration of videos recorded by this node in seconds. If it is non-zero
  videos that reach this duration will be stopped automatically.  This parameter is intended as a safety precaution
  against users starting a video and forgetting to stop it, and running out of disk space as a result.  Default: `0`
- if `record_metadata` is `true` a JSON file containing robot and camera pose information is generated alongside the
  PNG or AVI files. Default: `false`
- `camera_frame` is the frame ID of the camera. This is ignored unless `record_metadata` is `true`
- `enable_zoom` indicates whether or not the camera publishes its current zoom leven as a `std_msgs/Float64`. If it
  does, the zoom level is recorded in the metadata JSON file.

If `record_metadata` and `enable_zoom` are both `true` the node will subscribe to `zoom_level`.  Use `remap` as
appropriate to connect this subscription to the correct ROS topic for your camera's zoom level.

If the aspect ratio of the input images doesn't match the output video, the video will be letterboxed/pillarboxed as
appropriate.

The provided launch file will set `out_dir` to `$HOME` if it is run as a normal user. When launching the
`video_recorder_node` as part of a `robot_upstart` job it is possible that the `$HOME` envar is not defined, which is
why the default is `/tmp`.

The video file always uses OpenCV's `8UC3` matrix type in `BGR` format, regardless of the stream source's encoding.


Compressed Images
-------------------

If your camera only publishes `sensor_msgs/CompressedImage` data, or you have to use compressed images because of
bandwidth limitations, you can set the `~compressed` parameter to `true`.  This will force the node to subscribe to
compressed images instead of the default `sensor_msgs/Image` data.


Action Namespace
------------------

The `video_recorder_node` used `actionlib` to provide actions to save images, start, and stop recoridng video.  The
services are located in the image topic's namespace.
```xml
<node pkg="video_recorder" type="video_recorder_node" name="camera_recorder">
  <param name="topic" value="/camera/image_raw" />
  ...
</node>
```

This will result in the following action namespaces:
- `/camera/image_raw/save_image/`
- `/camera/image_raw/start_recording/`
- `/camera/image_raw/stop_recording/`


Recording Videos
------------------

StartRecording.action
```
string filename
uint64 duration
---
bool success
string path
---
uint32 time_elapsed
uint32 time_remaining
uint32 n_frames
```

Videos created with the `start_recording` action are XVID-encoded AVI files.

To start recording a video, use the `start_recording` action, optionally providing a filename for the
resulting .avi file and a recording duration.

If the `filename` parameter is empty, the default format of `YYYYMMDDhhmmss.avi` will be used.  If the `filename`
parameter is not empty, the `.avi` file extension should be included.  At present no other video encoding formats
are supported.

If the `duration` parameter is zero the video will record until `stop_recording` is called (see below). otherwise the
video will stop recording after the specified duration in seconds.

If `duration` is zero the action will generate no feedback. The video will record indefinitely until the corresponding
`stop_recording` action is called.

If `duration` is non-zero, feeback will be published at 1Hz, providing meta-data about the video including the elapsed
recording time, and time remaining.  Once the specified duration has elapsed the action will return the result and
stop recording automatically.


Stopping a Recording
-----------------------

StopRecording.action
```
bool arg
---
bool success
string path
uint64 size
uint64 duration
---
bool arg
```

To stop a recording in-progress, run the `stop_recording` action.  The `arg` parameter is a placeholder and its
actual value is ignored.

The `stop_recording` action produces no feedback and returns the result immediately.

Note that depending on the `fps` parameter set and the actual publishing rate of the camera, it's possible for the
video's actual duration to be different, but it will include all frames recorded during the specified duration in
real-time.  For example, if the camera itself publishes at 15FPS, but the `video_recorder_node` is configured with
an `fps` parameter of 30, a video recorded with a duration of 60 seconds will have a playback duration of only 30
seconds; the video file's FPS is twice that of the camera.  For this reason it is important to make sure the `fps`
parameter is properly set when launching the `video_recorder_node`.


Saving Images
---------------

SaveImage.action
```
string filename
uint32 delay
---
bool success
string path
---
uint32 time_remaining
```

To save a single frame as an image, use the `save_image` action, optionally specifying the filename
for the resulting image and a delay in seconds.

If the `filename` parameter is empty, the default format of `YYYYMMDDhhmmss.png` will be used.  If the `filename`
parameter is not empty, the file extension should be included in the parameter.  Supported extensions are:
- `.png` (default),
- `.jpg` (or `.jpeg`), and
- `.bmp`

The `delay` parameter is an optional delay to apply before taking the picture, measured in seconds.  This can be useful
to work around high-latency cameras to ensure the correct frame is recorded.

Feedback is provided at 1Hz and serves as a countdown timer.  Once the image has been saved to disk the action
returns the result.

Note that unlike videos, the size of the images saved is always the same as the published image topic.  No resizing,
letterboxing, nor pillarboxing is applied.


Is-Recording Indicator
-----------------------

The `video_recorder_node` publishes an `is_recording` topic which indicates whether or not video is currently being
recorded.  This topic publishes true/false at a rate equal to the FPS of the camera, as long as the node's status
is `>=2` (see below for status messages).  When the node is in the `WAITING` (1) status the `is_recording` topic is
not published.


Status
-------

The `video_recorder_node` also publishes its internal status at a rate of 1Hz on the `recorder_status` topic.

Status.msg
```
uint8 status
uint32 frames_received_last_second
uint32 frames_processed_last_second

```

The `status` field is a bit-field with the following meanings:
- 0-bit: Indicates that the node is waiting to receive data from the camera. Recording videos/saving images cannot be done
  in this state
- 1-bit: The node has received at least 1 frame from the camera and is running normally
- 2-bit: The node is currently recording video
- 4-bit: The node is running a countdown timer to save a single image

e.g.
A status of `0b00000010 = 2` indicates that the node is running normally, but is not recording.

e.g.
A status of `0b00000110 = 6` indicates that the node is running normally and recording video.

e.g.
A status of `0b00000001 = 1` indicates that the node is waiting to receive camera data

The `frames_received_last_second` and `frames_processed_last_second` indicate the number of raw frames received
from the camera in the last 1s of real-time and the number of frames written to video/image files.  Under normal
conditions, `frames_received_last_second` should be equal to the FPS of the camera.  When recording,
`frames_processed_last_second` should also match the camera's FPS.



audio_recorder
=================

This package contains the `audio_recorder_node`, which can be used to record a `.wav` file from an ALSA-compatible
input device.  This replicates some of the behaviour of packages found in `audio_common`, but wraps the functionality
in an `actionlib` server to make it compatible with Clearpath's GPS Navigation and IndoorNav software stacks.

The node's configuration is done via `rosparams`:
```yaml
audio_recorder_node:
  out_dir: /home/administrator/capture
  card: 0
  device: 0
  bitrate: 44100
  channels: 1
```
- `out_dir` is the directory on-disk to save the recorded files.  The directory is not created if it doesn't already
  exist on the disk. Default: `/tmp`
- `card` and `device` form part of the ALSA hardware device identifier.  For example, if `arecord -l` shows
  `card 1: U0x46d0x825 [USB Device 0x46d:0x825], device 0: USB Audio [USB Audio]` then use `card: 1` and
  `device: 0`.
- `bitrate` is the bitrate for ALSA to record audio from the device
- `channels` is the number of audio channels supported by the device.  Mono devices typically have 1 and stereo devices
  typically have 2, but this may vary depending on your hardware.

The provided launch file will set `out_dir` to `$HOME` if it is run as a normal user. When launching the
`audio_recorder_node` as part of a `robot_upstart` job it is possible that the `$HOME` envar is not defined, which is
why the default is `/tmp`.

Audio files are always saved as uncompressed .wav files.


Recording Audio
------------------

StartRecording.action
```
string filename
uint64 duration
---
bool success
string path
---
uint32 time_elapsed
uint32 time_remaining
```

Files created with the `start_recording` action are uncompressed `.wav` files.

To start recording, use the `start_recording` action, optionally providing a filename for the
resulting .avi file and a recording duration.

If the `filename` parameter is empty, the default format of `YYYYMMDDhhmmss.wav` will be used.  If the `filename`
parameter is not empty, the `.wav` file extension should be included.  At present no other audio formats
are supported.

If the `duration` parameter is zero the video will record until `stop_recording` is called (see below). otherwise the
file will stop recording after the specified duration in seconds.

If `duration` is zero the action will generate no feedback. The file will record indefinitely until the corresponding
`stop_recording` action is called.

If `duration` is non-zero, feeback will be published at 1Hz, providing meta-data about the audio including the elapsed
recording time, and time remaining.  Once the specified duration has elapsed the action will return the result and
stop recording automatically.


Stopping a Recording
-----------------------

StopRecording.action
```
bool arg
---
bool success
string path
uint64 duration
---
bool arg
```

To stop a recording in-progress, run the `stop_recording` action.  The `arg` parameter is a placeholder and its
actual value is ignored.

The `stop_recording` action produces no feedback and returns the result immediately.


Is-Recording Indicator
-----------------------

The `audio_recorder_node` publishes an `is_recording` topic which indicates whether or not audio is currently being
recorded.  This topic publishes true/false at a rate of 10Hz.


Status
-------

The `audio_recorder_node` also publishes its internal status at a rate of 10Hz on the `recorder_status` topic.

Status.msg
```
uint8 status

```

The `status` field is a bit-field with the following meanings:
- 1-bit: The node has started and is running
- 2-bit: The node is currently recording audio

e.g.
A status of `0b00000010 = 2` indicates that the node is running normally, but is not recording.

e.g.
A status of `0b00000110 = 6` indicates that the node is running normally and recording video.
