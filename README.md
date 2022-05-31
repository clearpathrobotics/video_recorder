video_recorder
=================

This package contains the `video_recorder_node`, which can be used to record a video file from
a topic publishing `sensor_msgs/Image` data.

The node's configuration is done via `rosparams`:
```yaml
video_recorder_node:
  fps: 30.0
  topic: /camera/image_raw
  out_dir: $HOME
```
- `topic` is fairly self-explanatory: this is the ROS topic to subscribe to. Default: `/camera/image_raw`
- `fps` is the FPS of the output file. This should be set to the same FPS that the camera node publihes at. Default: `30`
- `out_dir` is the directory on-disk to save the recorded videos.  The directory must already exist. Default: `$HOME`

The video file always uses OpenCV's `8UC3` matrix type in `BGR` format, regardless of the stream source's encoding.


Service Namespace
-------------------

The ROS services provided by the node use the `topic` parameter above as their namespace.  The following services
are provided:
- `/camera_topic/start_recording`
- `/camera_topic/stop_recording`
- `/camera_topic/save_image`

See below for details on how to use these services and the arguments they require.


Recording Videos
------------------

Videos created with the `start_recording` service are XVID-encoded AVI files.

To start recording a video, use the `start_recording` service, optionally providing a filename for the
resulting .avi file and a recording duration:
```bash
rosservice call  /camera/image_raw/start_recording "{filename: '', duration: 0}"
```
If the `filename` parameter is empty, the default format of `YYYYMMDDhhmmss.avi` will be used.  If the `filename`
parameter is not empty, the `.avi` file extension should be included.  At present no other video encoding formats
are supported.

If the `duration` parameter is zero the video will record until `stop_recording` is called (see below). otherwise the
video will stop recording after the specified duration in seconds.

To stop recording, use the `stop_recording` service:
```bash
rosservice call  /camera/image_raw/stop_recording "arg: false"
```
The `arg` parameter is ignored and is simply there to provide a placeholder parameter.

Note that depending on the `fps` parameter set and the actual publishing rate of the camera, it's possible for the
video's actual duration to be different, but it will include all frames recorded during the specified duration in
real-time.  For example, if the camera itself publishes at 15FPS, but the `video_recorder_node` is configured with
an `fps` parameter of 30, a video recorded with a duration of 60 seconds will have a playback duration of only 30
seconds; the video file's FPS is twice that of the camera.  For this reason it is important to make sure the `fps`
parameter is properly set when launching the `video_recorder_node`.


Saving Images
---------------

To save a single frame as an image, use the `save_image` service, optionally specifying the filename
for the resulting image:
```bash
rosservice call  /camera/image_raw/save_image "filename: ''"
```
If the `filename` parameter is empty, the default format of `YYYYMMDDhhmmss.png` will be used.  If the `filename`
parameter is not empty, the file extension should be included in the parameter.  Supported extensions are:
- `.png` (default),
- `.jpg` (or `.jpeg`), and
- `.bmp`
