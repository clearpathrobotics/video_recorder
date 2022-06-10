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
```
- `topic` is fairly self-explanatory: this is the ROS topic to subscribe to. Default: `/camera/image_raw`
- `fps` is the FPS of the output file. This should be set to the same FPS that the camera node publihes at. Default: `30`
- `output_height` is the frame height of the output video files
- `output_width` is the frame width of the output video files.
- `out_dir` is the directory on-disk to save the recorded videos.  The directory must already exist. Default: `/tmp`

If the aspect ratio of the input images doesn't match the output video, the video will be letterboxed/pillarboxed as
appropriate.

The provided launch file will set `out_dir` to `$HOME` if it is run as a normal user. When launching the
`video_recorder_node` as part of a `robot_upstart` job it is possible that the `$HOME` envar is not defined, which is
why the default is `/tmp`.

The video file always uses OpenCV's `8UC3` matrix type in `BGR` format, regardless of the stream source's encoding.


Action Namespace
------------------

The `video_recorder_node` used `actionlib` to provide actions to save images, start, and stop recoridng video.  The
services are located in the node's namespace.  For clarity we recommend launching the node in the same namespace
as the image topic:
```xml
<group ns="/camera/image_raw">
  <node pkg="video_recorder" type="video_recorder_node" name="capture">
    <!-- params -->
  </node>
</group>
```

This will result in the following action namespaces:
- `/camera/image_raw/capture/save_image/`
- `/camera/image_raw/capture/start_recording/`
- `/camera/image_raw/capture/stop_recording/`


Recording Videos
------------------

SaveImage.action
```
string filename
uint64 duration
---
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
