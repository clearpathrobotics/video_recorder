video_recorder
=================

This package contains the `video_recorder_node`, which can be used to record a video file from
a topic publishing `senso_msgs/Image` data.

The node's configuration is done via `rosparams`:
```yaml
video_recorder_node:
  fps: 30.0
  topic: /camera/image_raw
  out_dir: $HOME
```
- `topic` is fairly self-explanatory: this is the ROS topic to subscribe to.
- `fps` is the FPS of the output file. This should be set to the same FPS that the camera node publihes at.
- `out_dir` is the directory on-disk to save the recorded videos.  The directory must already exist.

The video file always uses OpenCV's `8UC3` matrix type in `BGR` format, regardless of the stream source's encoding.


Recording Videos
------------------

To start recording a video, use the `start_recording` service, optionally providing a filename for the
resulting .avi file and a recording duration:
```bash
rosservice call  /video_recorder_node/start_recording "{filename: '', duration: 0}"
```
If the `filename` argument is empty, the default format of `YYYYMMDDhhmmss.avi` will be used.  If the `duration`
parameter is zero the video will record until `stop_recording` is called (see below). otherwise the video will stop
recording after the specified duration in seconds.  (Note that depending on the `fps` parameter set and the actual
publishing rate of the camera, it's possible for the video's actual duration to be different, but it will include
all frames recorded during the specified duration in real-time.)

To stop recording, use the `stop_recording` service:
```bash
rosservice call  /video_recorder_node/stop_recording "arg: false"
```
The `arg` parameter is ignored and is simply there to provide a placeholder parameter.


Saving Images
---------------

To save a single frame as an image, use the `save_image` service, optionally specifying the filename
for the resulting image:
```bash
rosservice call  /video_recorder_node/save_image "filename: ''"
```
If the `filename` parameter is empty, the default format of `YYYYMMDDhhmmss.png` will be used.  If specifyng the
filename the extension should be specified.  Supported extensions are:
- `.png` (default),
- `.bmp`,
- `.jpg` and `.jpeg`
