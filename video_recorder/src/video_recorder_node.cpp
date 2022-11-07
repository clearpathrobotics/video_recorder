#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <thread>

#include <video_recorder/video_recorder_node.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace geometry_msgs;
using namespace video_recorder;
using namespace video_recorder_msgs;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// HELPER FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*!
 * Yes, this is a gross C function, but std::filesystem isn't available in C++14
 * and Melodic doesn't support C++17
 */
static unsigned long filesize(std::string path)
{
  struct stat buf;
  stat(path.c_str(), &buf);
  unsigned long size = buf.st_size;
  return size;
}

/*!
 * Shamelessly copied from https://stackoverflow.com/questions/874134/find-out-if-string-ends-with-another-string-in-c
 */
static inline bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

/*!
 * Resize a mat and apply letterbox/pillarbox lines to preserve the aspect ratio
 *
 * \param src  The image we're going to resize
 * \param dst  The image we're writing to. This image must be initialized to a black image of the correct size and type
 */
static void letterbox_or_pillarbox(const cv::Mat &src, cv::Mat &dst)
{
  double src_aspect = (double)src.cols / (double)src.rows;
  double dst_aspect = (double)dst.cols / (double)dst.rows;
  double scale;
  cv::Rect roi;

  if (src_aspect > dst_aspect)
  {
    // src is wider than dst -- dst must be letterboxed
    scale = (double)src.cols / (double)dst.cols;
    roi.width = dst.cols;
    roi.x = 0;
    roi.height = src.rows / scale;
    roi.y = (dst.rows - roi.height) / 2;
  }
  else if (src_aspect < dst_aspect)
  {
    // src is narrower than dst -- dst must be pillarboxed
    scale = (double)src.rows / (double)dst.rows;
    roi.width = src.cols / scale;
    roi.x = (dst.cols - roi.width) /2;
    roi.height = dst.rows;
    roi.y = 0;
  }
  else
  {
    // same aspect; no bars needed
    scale = (double)src.cols / (double)dst.cols;
    roi.width = dst.cols;
    roi.x = 0;
    roi.height = dst.rows;
    roi.y = 0;
  }

  cv::resize(src, dst(roi), roi.size());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// VideoRecorderNode
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*!
 * Creates the start/stop services, loads the ROS parameters, subscribes to the input topic
 *
 * \param nh              The NodeHandle for this node
 * \param img_topic       The image topic to subscribe to. Must be sensor_msgs/Image or sensor_msgs/CompressedImage
 * \param out_dir         The directory where images and videos are saved
 * \param camera_frame    The frame the camera records in
 * \param fps             The baseline FPS of the camera topic
 * \param output_height   The height of the recorded video files in pixels
 * \param output_width    The width of the recorded video files in pixels
 * \param compressed      Is the image topic a compressed image, or raw image?
 * \param record_metadata Should we record additional meta-data about the robot's state in a CSV along with the
 * \param supports_zoom   Does the camera support zooming?
 *                        photo/video file?
 */
VideoRecorderNode::VideoRecorderNode(
  ros::NodeHandle &nh,
  const std::string &img_topic,
  const std::string &out_dir,
  const std::string &camera_frame,
  const double fps,
  const double output_height,
  const double output_width,
  const bool compressed,
  const bool record_metadata,
  const bool supports_zoom
) :
  nh_(nh),
  img_topic_(img_topic),
  out_dir_(out_dir),
  camera_frame_(camera_frame),
  fps_(fps),
  output_height_(output_height),
  output_width_(output_width),
  compressed_(compressed),
  record_metadata_(record_metadata),
  supports_zoom_(supports_zoom),
  frame_service_(nh, img_topic + "/save_image",      boost::bind(&VideoRecorderNode::saveImageHandler, this, _1), false),
  start_service_(nh, img_topic + "/start_recording", boost::bind(&VideoRecorderNode::startRecordingHandler, this, _1), false),
  stop_service_(nh, img_topic + "/stop_recording",   boost::bind(&VideoRecorderNode::stopRecordingHandler, this, _1), false)
{
  is_recording_ = std_msgs::Bool();
  is_recording_.data = false;
  capture_next_frame_ = false;
  vout_ = NULL;

  status_.status = 0x00;
  status_.frames_received_last_second = 0;
  status_.frames_processed_last_second = 0;

  int max_duration_seconds;
  nh.param<int>("max_duration", max_duration_seconds, 0);
  if (max_duration_seconds < 0)
    max_duration_seconds = 0;
  max_video_duration_ = std::chrono::seconds((unsigned long)max_duration_seconds);

  pthread_mutex_init(&video_recording_lock_, NULL);

  is_recording_pub_ = nh.advertise<std_msgs::Bool>(img_topic + "/is_recording", 1);
  status_pub_ = nh.advertise<video_recorder_msgs::Status>(img_topic + "/recorder_status", 1);

  zoom_level_ = 0.0;
  if (supports_zoom_)
    zoom_sub_ = nh.subscribe("zoom_level", 1, &VideoRecorderNode::zoomLevelCallback, this);

  // subscribe to either the raw sensor_msgs/Image or sensor_msgs/CompressedImage topic as needed
  if (!compressed_)
    img_sub_ = nh.subscribe(img_topic_, 1, &VideoRecorderNode::imageCallback, this);
  else
    img_sub_ = nh.subscribe(img_topic_, 1, &VideoRecorderNode::compressedImageCallback, this);

  frame_service_.start();
  start_service_.start();
  stop_service_.start();

  pthread_create(&status_thread_, NULL, &statusPublisher, this);
}

/*!
 * Stops recording if we are currently saving a video and closes any open files
 */
VideoRecorderNode::~VideoRecorderNode()
{
  stopRecording();
  pthread_join(status_thread_, NULL);
}

/*!
 * Publish the state of the node at a rate of 1Hz
 */
void *VideoRecorderNode::statusPublisher(void *arg)
{
  VideoRecorderNode *node = (VideoRecorderNode*)arg;

  node->status_.status = video_recorder_msgs::Status::WAITING;

  ros::Rate rate(1);
  while (ros::ok())
  {
    rate.sleep();

    // Check if we're recording and set the flag accordingly
    // The photo-timer flag is set inside the save_image action handler
    if (node->is_recording_.data)
      node->status_.status |= video_recorder_msgs::Status::RECORDING;
    else
      node->status_.status &= ~video_recorder_msgs::Status::RECORDING;

    // Publish and reset the counters
    node->status_pub_.publish(node->status_);
    node->status_.frames_received_last_second = 0;
    node->status_.frames_processed_last_second = 0;
  }

  return NULL;
}

/*!
 * Create a filename with the YYYYMMDDhhmmss.{extension} format.
 *
 * \param extension The extension for the filename WITHOUT the leading "." charater. e.g. "png" or "avi"
 */
std::string VideoRecorderNode::defaultFilename(std::string extension)
{
  std::stringstream ss;
  char time_str [80];
  std::time_t rawtime;
  std::tm* timeinfo;
  std::time(&rawtime);
  timeinfo = std::localtime(&rawtime);
  std::strftime(time_str,80,"%Y%m%d%H%M%S",timeinfo);
  std::puts(time_str);
  ss << time_str << "." << extension;

  std::string result;
  ss >> result;
  return result;
}

/*!
 * Handler for the StartRecording service.  Initializes the cv::VideoWriter instance, sets is_recording_ to true,
 * saves the start time of the recording.
 * Will return an error if we are already recording video
 *
 * \param goal  The action goal we've received
 */
void VideoRecorderNode::startRecordingHandler(const video_recorder_msgs::StartRecordingGoalConstPtr& goal)
{
  video_recorder_msgs::StartRecordingResult result;
  if (!(status_.status & video_recorder_msgs::Status::RUNNING))
  {
    ROS_WARN("Unable to start recording; no data received from the camera yet");
    result.path = "";
    result.success = false;
    start_service_.setAborted(result, "Unable to start recording; no data received yet");
  }
  else if (!is_recording_.data)
  {
    // First figure out the full path of the .avi file we're saving
    std::stringstream ss;
    ss << out_dir_;
    if (goal->filename.length() == 0)
    {
      ss << defaultFilename("avi");
    }
    else
    {
      ss << goal->filename;
      if (!ends_with(goal->filename, ".avi"))
      {
        ss << ".avi";
      }
    }
    ss >> video_path_;

    if (record_metadata_)
      recordMetadata(video_path_);

    // create the video_writer
    ROS_INFO("Recording to %s for %d seconds (0=inf)", video_path_.c_str(), (int)goal->duration);
    pthread_mutex_lock(&video_recording_lock_);
    // record the start time of the recording and max duration
    desired_video_duration_ = std::chrono::seconds(goal->duration);
    video_start_time_ = std::chrono::system_clock::now();
    n_frames_ = 0;
    vout_ = createVideoWriter();
    is_recording_.data = true;
    pthread_mutex_unlock(&video_recording_lock_);

    // publish feedback while we're recording if we specified a duration
    ros::Rate rate(10);
    video_recorder_msgs::StartRecordingFeedback feedback;
    while (is_recording_.data && goal->duration > 0 && !start_service_.isPreemptRequested())
    {
      auto now = std::chrono::system_clock::now();
      auto elapsed = now - video_start_time_;
      auto remaining = desired_video_duration_ - elapsed;

      feedback.time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
      feedback.time_remaining = std::chrono::duration_cast<std::chrono::seconds>(remaining).count();;
      feedback.n_frames = n_frames_;

      start_service_.publishFeedback(feedback);
      rate.sleep();
    }

    if (start_service_.isPreemptRequested())
    {
      ROS_WARN("Recording cancelled!");
      stopRecording();
      result.path = video_path_;
      result.success = false;
      start_service_.setAborted(result, "User cancelled fixed-duration video");
    }

    // return the result
    result.path = video_path_;
    result.success = true;
    start_service_.setSucceeded(result);
  }
  else
  {
    ROS_WARN("Unable to start recording; node is already recording to %s", video_path_.c_str());
    result.path = "";
    result.success = false;
    start_service_.setAborted(result, "Unable to start recording; node is already recording");
  }
}

/*!
 * Handler for the StopRecording service. Stops any video recording in process, even if
 * we have not yet reached the duration specified in the argument to StartRecordingHandler.
 */
void VideoRecorderNode::stopRecordingHandler(const video_recorder_msgs::StopRecordingGoalConstPtr &goal)
{
  video_recorder_msgs::StopRecordingResult result;
  if (is_recording_.data)
  {
    ROS_INFO("Stopping recording to to %s", video_path_.c_str());

    pthread_mutex_lock(&video_recording_lock_);
    stopRecording();
    pthread_mutex_unlock(&video_recording_lock_);


    // calculate the total recording time in seconds
    auto stop_time = std::chrono::system_clock::now();
    auto elapsed = stop_time - video_start_time_;
    unsigned long seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

    result.duration = seconds;
    result.path = video_path_;
    result.size = filesize(video_path_);
    result.success = true;
    stop_service_.setSucceeded(result);
  }
  else
  {
    ROS_WARN("Unable to stop recording; node is not recording");
    result.duration = 0;
    result.path = "";
    result.size = 0;
    result.success = false;
    stop_service_.setAborted(result, "Unable to stop recording; node is not recording");
  }
}

/*!
 * Handler for the SaveImage action. Delays the specified number of seconds and then flags that the next frame
 * should be captured and saved to a file
 */
void VideoRecorderNode::saveImageHandler(const video_recorder_msgs::SaveImageGoalConstPtr& goal)
{
  video_recorder_msgs::SaveImageResult result;
  if (!(status_.status & video_recorder_msgs::Status::RUNNING))
  {
    ROS_WARN("Unable to save image; no data received from the camera yet");
    result.path = "";
    result.success = false;
    frame_service_.setAborted(result, "Unable to save image; no data received yet");
  }
  else if (!capture_next_frame_)
  {
    // First figure out the full path of the .avi file we're saving
    std::stringstream ss;
    ss << out_dir_;
    if (goal->filename.length() == 0)
    {
      ss << defaultFilename("png");
    }
    else
    {
      ss << goal->filename;
      // if the user specified a file extension, try to use that if we know what it is
      // otherwise use .png
      if (!ends_with(goal->filename, ".bmp") &&
          !ends_with(goal->filename, ".jpg") &&
          !ends_with(goal->filename, ".jpeg") &&
          !ends_with(goal->filename, ".png")
          // TODO: any more common formats we want to be able to support?
      )
      {
        ss << ".png";
      }
    }
    ss >> image_path_;

    ros::Rate r(1);
    unsigned long time_remaining = goal->delay;
    ROS_INFO("Saving image in %d seconds to %s", goal->delay, image_path_.c_str());

    video_recorder_msgs::SaveImageFeedback feedback;
    for(unsigned long i=0; i<time_remaining; i++)
    {
      feedback.time_remaining = time_remaining-i;
      frame_service_.publishFeedback(feedback);
      status_.status |= video_recorder_msgs::Status::PHOTO_TIMER;
      r.sleep();
    }
    image_saved_ = false;
    capture_next_frame_ = true;

    while (!image_saved_)
      r.sleep();

    result.path = image_path_;
    result.success = true;
    frame_service_.setSucceeded(result);
    status_.status &= ~video_recorder_msgs::Status::PHOTO_TIMER;
  }
  else
  {
    ROS_WARN("Already queued to record the next frame");
    result.path = "";
    result.success = false;
    frame_service_.setAborted(result, "SaveImage has already been requested");
  }
}

/*!
 * Callback for the zoom_level subscription. Sets the zoom_level_ variable
 * according to the provided argument
 */
void VideoRecorderNode::zoomLevelCallback(const std_msgs::Float64 &zoom)
{
  zoom_level_ = zoom.data;
}

/*!
 * Subscription to the image topic we're responsible for capturing.  As long as the node is alive
 * we keep an open subscription, but we only process the frame if we're either recording or about to
 * save a still image.
 */
void VideoRecorderNode::imageCallback(const sensor_msgs::Image &img)
{
  // clear the waiting flag, set the running flag, increment the frames received
  status_.status &= ~video_recorder_msgs::Status::WAITING;
  status_.status |= video_recorder_msgs::Status::RUNNING;
  status_.frames_received_last_second++;

  if (is_recording_.data || capture_next_frame_)
  {
    cv::Mat m;
    bool conversion_ok = image2mat(img, m);

    // Just kick out early if the conversion fails
    if(!conversion_ok)
      return;

    processImage(m);
  }
  is_recording_pub_.publish(is_recording_);
}

/*!
 * Subscription to the compressed image topic, used when ~compressed is true.
 * We only bother processing the frame if we're going to write it to the video or image files
 */
void VideoRecorderNode::compressedImageCallback(const sensor_msgs::CompressedImage &img)
{
  // clear the waiting flag, set the running flag, increment the frames received
  status_.status &= ~video_recorder_msgs::Status::WAITING;
  status_.status |= video_recorder_msgs::Status::RUNNING;
  status_.frames_received_last_second++;

  if (is_recording_.data || capture_next_frame_)
  {
    cv::Mat m = cv::imdecode(img.data, cv::IMREAD_UNCHANGED);
    processImage(m);
  }
  is_recording_pub_.publish(is_recording_);
}

/*!
 * Backend for imageCallback and compressedImageCallback to reduce duplication
 */
void VideoRecorderNode::processImage(const cv::Mat &m)
{
  status_.frames_processed_last_second++;
  if (is_recording_.data)
  {
    appendFrame(m);
  }

  if (capture_next_frame_)
  {
    if (record_metadata_)
      recordMetadata(image_path_);
    saveImage(m);
  }
}

/*!
 * We can't just create the VideoWriter when we call startRecordingHandler because the size of the frame is
 * not yet known. This is called inside the image subscription handler to create the video writer on the first
 * frame of every video we create.
 * The created object is destroyed when we call stopRecording()
 */
cv::VideoWriter *VideoRecorderNode::createVideoWriter()
{
  return new cv::VideoWriter(video_path_, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps_,
                             cv::Size(output_width_, output_height_), true);
}

void VideoRecorderNode::appendFrame(const cv::Mat &img)
{
  cv::Mat resized = cv::Mat::zeros(output_height_, output_width_, CV_8UC3);
  letterbox_or_pillarbox(img, resized);
  pthread_mutex_lock(&video_recording_lock_);
  *(vout_) << resized;
  n_frames_++;

  // check if we should stop recording
  auto now = std::chrono::system_clock::now();
  auto elapsed = now - video_start_time_;
  if (max_video_duration_ > std::chrono::seconds(0) && elapsed >= max_video_duration_)
  {
    ROS_WARN("Video recording reached hard time limit. Stopping recording %s", video_path_.c_str());
    stopRecording();
  }
  else if (desired_video_duration_ > std::chrono::seconds(0) && elapsed >= desired_video_duration_)
  {
    ROS_INFO("User-specified duration elapsed; stopping recording %s", video_path_.c_str());
    stopRecording();
  }
  pthread_mutex_unlock(&video_recording_lock_);
}

/*!
 * Close the video file and destroy the cv::VideoWriter instance
 */
void VideoRecorderNode::stopRecording()
{
  is_recording_.data = false;
  if (vout_ != NULL)
  {
    vout_->release();
    delete vout_;
    vout_ = NULL;
  }
}

void VideoRecorderNode::saveImage(const cv::Mat &img)
{
  capture_next_frame_ = false;
  cv::imwrite(image_path_, img);
  image_saved_ = true;
}

/*!
 * Converts the raw sensor image into a BGR8 cv::Mat object
 */
bool VideoRecorderNode::image2mat(const sensor_msgs::Image &src, cv::Mat &dst)
{
  // realsense2_camera seems to have a bug where it uses the OpenCV encoding, which breaks the conversion
  // so make a shallow copy and ensure the encoding is correct
  sensor_msgs::Image img_fixed;
  img_fixed.header = src.header;
  img_fixed.is_bigendian = src.is_bigendian;
  img_fixed.height = src.height;
  img_fixed.width = src.width;
  img_fixed.step = src.step;
  img_fixed.data = src.data;
  if (src.encoding == "16UC1")
  {
    img_fixed.encoding = sensor_msgs::image_encodings::MONO16;
  }
  else if(src.encoding == "8UC1")
  {
    img_fixed.encoding = "mono8";
  }
  else
  {
    img_fixed.encoding = src.encoding;
  }

  cv_bridge::CvImagePtr cv_ptr;
  bool frame_ok = true;
  try
  {
    // depending on the input image encoding we may need to convert the colour to something usable
    // whatever we get, convert it to BGR8, OpenCV's default 24-bit RGB encoding
    cv_ptr = cv_bridge::toCvCopy(img_fixed, img_fixed.encoding);
    if (img_fixed.encoding == sensor_msgs::image_encodings::RGB8)
    {
      cv::cvtColor(cv_ptr->image, dst, cv::COLOR_RGB2BGR);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::RGBA8)
    {
      cv::cvtColor(cv_ptr->image, dst, cv::COLOR_RGBA2BGR);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::RGB16)
    {
      cv::cvtColor(cv_ptr->image, dst, cv::COLOR_RGBA2BGR);
      dst.convertTo(dst, CV_8UC3);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::RGBA16)
    {
      cv::cvtColor(cv_ptr->image,dst, cv::COLOR_RGB2BGR);
      dst.convertTo(dst, CV_8UC3);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::BGR8)
    {
      cv_ptr->image.copyTo(dst);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::BGRA8)
    {
      cv::cvtColor(cv_ptr->image, dst, cv::COLOR_BGRA2BGR);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::BGR16)
    {
      cv_ptr->image.convertTo(dst, CV_8UC3);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::BGRA16)
    {
      cv::cvtColor(cv_ptr->image, dst, cv::COLOR_BGRA2BGR);
      dst.convertTo(dst, CV_8UC3);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::MONO8)
    {
      cv::cvtColor(cv_ptr->image, dst, cv::COLOR_GRAY2BGR);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::MONO16)
    {
      cv::cvtColor(cv_ptr->image, dst, cv::COLOR_GRAY2BGR);
      dst.convertTo(dst, CV_8UC3, 1.0/255.0);  // only take the high byte, otherwise everything is completely washed out
    }
    else
    {
      ROS_WARN("Unsupported image format %s", img_fixed.encoding.c_str());
      frame_ok = false;
    }
  }
  catch(cv_bridge::Exception &err)
  {
    ROS_ERROR("OpenCV Error %s", err.what());
    frame_ok = false;
  }

  return frame_ok;
}

/*!
 * Record the current time, position on the map, and camera frame position to a CSV file
 *
 * \param filename  The base filename for the resulting meta-data file
 */
void VideoRecorderNode::recordMetadata(const std::string &filename)
{
  ROS_INFO("Recording metadata for %s...", filename.c_str());

  // get the current camera pose and robot position on the map
  geometry_msgs::Twist pos_map = lookupTransform("map", "base_link");
  geometry_msgs::Twist pos_cam = lookupTransform("base_link", camera_frame_);

  // get the current time
  // ctime adds a newline, so remove it
  auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  char *time_str = std::ctime(&now);
  if (time_str[strlen(time_str)-1] == '\n')
    time_str[strlen(time_str)-1] = '\0';

  std::string zoom_lvl;
  if (supports_zoom_)
  {
    std::stringstream ss;
    ss << zoom_level_;
    ss >> zoom_lvl;
  }
  else
  {
    zoom_lvl = "null";
  }

  std::ofstream fout(filename+".json");
  fout << "{" << std::endl
       << "  \"time\": \"" << time_str << "\"," << std::endl
       << "  \"file\": \"" << filename << "\"," << std::endl
       << "  \"robot_pose\": {" << std::endl
       << "    \"linear\": {" << std::endl
       << "      \"x\": " << pos_map.linear.x << "," << std::endl
       << "      \"y\": " << pos_map.linear.y << "," << std::endl
       << "      \"z\": " << pos_map.linear.z << std::endl
       << "    }," << std::endl
       << "    \"angular\": {" << std::endl
       << "      \"x\": " << pos_map.angular.x << "," << std::endl
       << "      \"y\": " << pos_map.angular.y << "," << std::endl
       << "      \"z\": " << pos_map.angular.z << std::endl
       << "    }" << std::endl
       << "  }," << std::endl
       << "  \"camera_pose\": {" << std::endl
       << "    \"linear\": {" << std::endl
       << "      \"x\": " << pos_cam.linear.x << "," << std::endl
       << "      \"y\": " << pos_cam.linear.y << "," << std::endl
       << "      \"z\": " << pos_cam.linear.z << std::endl
       << "    }," << std::endl
       << "    \"angular\": {" << std::endl
       << "      \"x\": " << pos_cam.angular.x << "," << std::endl
       << "      \"y\": " << pos_cam.angular.y << "," << std::endl
       << "      \"z\": " << pos_cam.angular.z << std::endl
       << "    }" << std::endl
       << "  }," << std::endl
       << "  \"zoom\":" << zoom_lvl.c_str() << std::endl
       << "}" << std::endl;
  fout.close();

  ROS_INFO("Location on map when saving %s: %f %f %f %f %f %f",
    filename.c_str(), pos_map.linear.x, pos_map.linear.y, pos_map.linear.z,
    pos_map.angular.x, pos_map.angular.y, pos_map.angular.z);
  ROS_INFO("Camera position when saving %s: %f %f %f %f %f %f",
    filename.c_str(), pos_cam.linear.x, pos_cam.linear.y, pos_cam.linear.z,
    pos_cam.angular.x, pos_cam.angular.y, pos_cam.angular.z);
}

/*!
 * Get the current/newest tf between two frames
 *
 * \param target_frame  The target frame to look up
 * \param fixed_frame   The fixed frame we're looking of target_frame in
 *
 * \return The position of target_frame relative to fixed_frame
 */
geometry_msgs::Twist VideoRecorderNode::lookupTransform(const std::string &target_frame, const std::string &fixed_frame)
{
  tf2_ros::Buffer tf_buf(ros::Duration(2.0));
  tf2_ros::TransformListener tf_listener(tf_buf);

  geometry_msgs::TransformStamped tf_stamped = tf_buf.lookupTransform(
    target_frame, fixed_frame, ros::Time(0), ros::Duration(5.0));

  geometry_msgs::Twist result;
  result.linear.x = tf_stamped.transform.translation.x;
  result.linear.y = tf_stamped.transform.translation.y;
  result.linear.z = tf_stamped.transform.translation.z;

  tf::Quaternion rotq(tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z, tf_stamped.transform.rotation.w);
  tf::Matrix3x3 m(rotq);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  result.angular.x = roll;
  result.angular.y = pitch;
  result.angular.z = yaw;

  return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MAIN
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*!
 * Checks if a file or directory exists on-disk
 *
 * \return  True if the path exists on-disk, otherwise false
 */
static bool fileExists(const std::string &path)
{
  struct stat buf;
  return stat(path.c_str(), &buf) == 0;
}

/*!
 * Checks if a given path on-disk exists AND is a directory, not a file
 *
 * \return True if the path is a directory that exists, otherwise false
 */
static bool isDirectory(const std::string &path)
{
  struct stat buf;
  if (stat(path.c_str(), &buf) != 0)
  {
    return false;
  }
  else if (!(buf.st_mode & S_IFDIR))
  {
    return false;
  }
  else
  {
    return true;
  }
}

/*!
 * Creates the ouput directory on-disk if it doesn't already exist
 * Exits with code 1 if there is an error creating the drectory for any reason
 *
 * \param dir  The output directory to create. May be nested, e.g. /foo/bar/.  Must end with a / character
 */
static void createOutputDirectory(const std::string &dir)
{
  if (fileExists(dir) && isDirectory(dir))
  {
    // everything is ok!
    ROS_INFO("%s is a valid output directory", dir.c_str());
  }
  else if (fileExists(dir) && !isDirectory(dir))
  {
    // dir is a file, not a directory; this is an error!
    ROS_ERROR("%s is a file not a directory!", dir.c_str());
    exit(1);
  }
  else
  {
    // the directory doesn't exist! create it
    //
    size_t start_at = 1;                // skip the leading /
    while (start_at < dir.length()-1)
    {
      size_t pos = dir.find("/", start_at);
      std::string substr = dir.substr(0, pos);

      if (fileExists(substr) && !isDirectory(substr))
      {
        // we've found a file mid-path!
        ROS_ERROR("%s is a file, not a directory!", substr.c_str());
        exit(1);
      }
      else if (!isDirectory(substr))
      {
        ROS_WARN("Creating %s", substr.c_str());
        int ret = mkdir(substr.c_str(), 0755);
        if (ret != 0)
        {
          ROS_ERROR("Failed to create directory %s: error %d", substr.c_str(), ret);
          exit(1);
        }
      }

      start_at = pos+1;
    }
  }
}

/*!
 * Create the node and spin. Nothing fancy to see here.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_recorder_node");
  ros::NodeHandle nh("~");

  std::string img_topic;
  std::string out_dir;
  std::string camera_frame;
  double fps;
  int output_height;
  int output_width;
  bool compressed;
  bool record_metadata;
  bool supports_zoom;

  nh.param<std::string>("topic", img_topic, "/camera/image_raw");
  nh.param<std::string>("out_dir", out_dir, "/tmp");
  nh.param<std::string>("camera_frame", camera_frame, "camera");
  nh.param<double>("fps", fps, 30.0);
  nh.param<int>("output_height", output_height, 480);
  nh.param<int>("output_width", output_width, 640);
  nh.param<bool>("compressed", compressed, false);
  nh.param<bool>("record_metadata", record_metadata, false);
  nh.param<bool>("enable_zoom", supports_zoom, false);

  // ensure the output directory ends with a / and create it if it doesn't already exist!
  if (out_dir[out_dir.length()-1] != '/')
  {
    out_dir.push_back('/');
  }
  createOutputDirectory(out_dir);

  VideoRecorderNode node(nh, img_topic, out_dir, camera_frame,
                         fps, output_height, output_width,
                         compressed, record_metadata, supports_zoom);
  ros::spin();
  return 0;
}
