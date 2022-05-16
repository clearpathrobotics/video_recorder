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
  encoding: bgr8
```
- `topic` is fairly self-explanatory: this is the ROS topic to subscribe to.
- `fps` is the FPS of the output file. This should be set to the same FPS that the camera node publihes at.
- `out_dir` is the directory on-disk to save the recorded videos.  The directory must already exist.
- `encoding` is the encoding of the image topic. Most RGB cameras will use `bgr8`.

Note: there is currently a bug when trying to use this node to record 16-bit grayscale data (e.g. raw depth data
from a RealSense or Zed2 camera).

Recording Videos
------------------

To start recording a video, use the `start_recording` service, optionally providing a filename for the
resulting .avi file:
```bash
rosservice call  /video_recorder_node/start_recording "filename: ''"
```
If the `filename` argument is empty, the default format of `YYYYMMDDhhmmss.avi` will be used.

To stop recording, use the `stop_recording` service:
```bash
rosservice call  /video_recorder_node/stop_recording "arg: false"
```
The `arg` parameter is ignored and is simply there to provide a placeholder parameter.

The provided launch file will run the `video_recorder_node` inside the same namespace as the subscribed topic.
