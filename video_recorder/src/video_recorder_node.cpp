#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <thread>
#include <chrono>

#include <video_recorder/video_recorder_node.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

static const rmw_qos_profile_t rmw_qos_profile_latch =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// VideoRecorderNode
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace video_recorder
{

VideoRecorderNode::VideoRecorderNode(std::shared_ptr<rclcpp::Node> node)
  : node_(node)
{
  using namespace std::placeholders;

  node_->declare_parameter("topic", "camera/image_raw");
  node_->declare_parameter("out_dir", "/tmp");
  node_->declare_parameter("mount_path", "");
  node_->declare_parameter("camera_frame", "camera");
  node_->declare_parameter("fps", 30.0);
  node_->declare_parameter("output_height", 480);
  node_->declare_parameter("output_width", 640);
  node_->declare_parameter("compressed", false);
  node_->declare_parameter("record_metadata", false);
  node_->declare_parameter("enable_zoom", false);
  node_->declare_parameter("max_duration", 0);

  int max_duration_seconds;
  node_->get_parameter("topic", img_topic_);
  node_->get_parameter("out_dir", out_dir_);
  node_->get_parameter("mount_path", mount_path_);
  node_->get_parameter("camera_frame", camera_frame_);
  node_->get_parameter("fps", fps_);
  node_->get_parameter("output_height", output_height_);
  node_->get_parameter("output_width", output_width_);
  node_->get_parameter("compressed", compressed_);
  node_->get_parameter("record_metadata", record_metadata_);
  node_->get_parameter("enable_zoom", supports_zoom_);
  node_->get_parameter("max_duration", max_duration_seconds);

  // ensure the output directory ends with a / and create it if it doesn't already exist!
  if (out_dir_[out_dir_.length()-1] != '/')
  {
    out_dir_.push_back('/');
  }
  createOutputDirectory(out_dir_);

  // if we've specified a mount path, make sure it ends with a / too
  if (mount_path_.length() > 0 && mount_path_[mount_path_.length()-1] != '/')
  {
    mount_path_.push_back('/');
  }

  is_recording_ = std_msgs::msg::Bool();
  is_recording_.data = false;
  capture_next_frame_ = false;
  vout_ = NULL;

  status_.status = 0x00;
  status_.frames_received_last_second = 0;
  status_.frames_processed_last_second = 0;

  if (max_duration_seconds < 0)
  {
    max_duration_seconds = 0;
  }
  max_video_duration_ = std::chrono::seconds((unsigned long)max_duration_seconds);

  pthread_mutex_init(&video_recording_lock_, NULL);

  auto qos_latch = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_latch), rmw_qos_profile_latch);
  is_recording_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(img_topic_ + "/is_recording", qos_latch);
  status_publisher_ = node_->create_publisher<video_recorder_msgs::msg::Status>(img_topic_ + "/recorder_status", 1);

  zoom_level_ = 0.0;
  if (supports_zoom_)
    zoom_subscriber_ = node_->create_subscription<std_msgs::msg::Float64>("zoom_level", 1, std::bind(&VideoRecorderNode::zoomLevelCallback, this,_1));

  // subscribe to either the raw sensor_msgs/Image or sensor_msgs/CompressedImage topic as needed
  if (!compressed_)
    img_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(img_topic_, 1, std::bind(&VideoRecorderNode::imageCallback, this, _1));
  else
    img_compressed_subscriber_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(img_topic_, 1, std::bind(&VideoRecorderNode::compressedImageCallback, this, _1));

  // Initialize actions
  auto saveImageGoalHandler = [this](
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const SaveImageAction::Goal> goal)
  {
    RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  auto saveImageCancelHandler = [this](
    const std::shared_ptr<GoalHandleSaveImage> goal_handle)
  {
    RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  auto saveImageAcceptedHandler = [this](
    const std::shared_ptr<GoalHandleSaveImage> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor,
    // so we declare a lambda function to be called inside a new thread
    auto execute_in_thread = [this, goal_handle](){return this->executeSaveImage(goal_handle);};
    std::thread{execute_in_thread}.detach();
  };
  frame_server_ = rclcpp_action::create_server<SaveImageAction>(
    node_,
    img_topic_ + "/save_image",
    saveImageGoalHandler,
    saveImageCancelHandler,
    saveImageAcceptedHandler);

  auto startRecordingGoalHandler = [this](
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const StartRecordingAction::Goal> goal)
  {
    RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  auto startRecordingCancelHandler = [this](
    const std::shared_ptr<GoalHandleStartRecording> goal_handle)
  {
    RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  auto startRecordingAcceptedHandler = [this](
    const std::shared_ptr<GoalHandleStartRecording> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor,
    // so we declare a lambda function to be called inside a new thread
    auto execute_in_thread = [this, goal_handle](){return this->executeStartRecording(goal_handle);};
    std::thread{execute_in_thread}.detach();
  };
  start_server_ = rclcpp_action::create_server<StartRecordingAction>(
    node_,
    img_topic_ + "/start_recording",
    startRecordingGoalHandler,
    startRecordingCancelHandler,
    startRecordingAcceptedHandler);

  auto stopRecordingGoalHandler = [this](
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const StopRecordingAction::Goal> goal)
  {
    RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  auto stopRecordingCancelHandler = [this](
    const std::shared_ptr<GoalHandleStopRecording> goal_handle)
  {
    RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  auto stopRecordingAcceptedHandler = [this](
    const std::shared_ptr<GoalHandleStopRecording> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor,
    // so we declare a lambda function to be called inside a new thread
    auto execute_in_thread = [this, goal_handle](){return this->executeStopRecording(goal_handle);};
    std::thread{execute_in_thread}.detach();
  };
  stop_server_ = rclcpp_action::create_server<StopRecordingAction>(
    node_,
    img_topic_ + "/stop_recording",
    stopRecordingGoalHandler,
    stopRecordingCancelHandler,
    stopRecordingAcceptedHandler);

  // publish once to start the latch
  is_recording_publisher_->publish(is_recording_);

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

  node->status_.status = video_recorder_msgs::msg::Status::WAITING;

  rclcpp::Rate rate(1);
  while (rclcpp::ok())
  {
    rate.sleep();

    // Check if we're recording and set the flag accordingly
    // The photo-timer flag is set inside the save_image action handler
    if (node->is_recording_.data)
      node->status_.status |= video_recorder_msgs::msg::Status::RECORDING;
    else
      node->status_.status &= ~video_recorder_msgs::msg::Status::RECORDING;

    // Publish and reset the counters
    node->status_publisher_->publish(node->status_);
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
  std::strftime(time_str,80,"%Y-%m-%d_%H-%M-%S",timeinfo);
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
void VideoRecorderNode::executeStartRecording(const std::shared_ptr<GoalHandleStartRecording> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Executing start recording action goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<StartRecordingAction::Feedback>();
  auto result = std::make_shared<StartRecordingAction::Result>();

  if (!(status_.status & video_recorder_msgs::msg::Status::RUNNING))
  {
    RCLCPP_WARN(rclcpp::get_logger("VideoRecorder"), "Unable to start recording; no data received from the camera yet");
    result->header.stamp = node_->now();
    result->success = false;
    result->path = video_result_path_;
    goal_handle->abort(result);
  }
  else if (!is_recording_.data)
  {
    // First figure out the full path of the .avi file we're saving
    std::stringstream video_path_ss;
    std::stringstream result_path_ss;
    video_path_ss << out_dir_;
    if (mount_path_.length() > 0)
      result_path_ss << mount_path_;
    else
      result_path_ss << out_dir_;
    if (goal->filename.length() == 0)
    {
      std::string default_filename = defaultFilename("avi");
      video_path_ss << default_filename;
      result_path_ss << default_filename;
    }
    else
    {
      video_path_ss << goal->filename;
      result_path_ss << goal->filename;
      if (!ends_with(goal->filename, ".avi"))
      {
        video_path_ss << ".avi";
        result_path_ss << ".avi";
      }
    }
    video_path_ss >> video_path_;
    result_path_ss >> video_result_path_;

    // record the max duration & start recording
    RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Recording to %s for %d seconds (0=inf)", video_path_.c_str(), (int)goal->duration);
    desired_video_duration_ = std::chrono::seconds(goal->duration);
    startRecording();

    // publish feedback while we're recording if we specified a duration
    rclcpp::Rate rate(10);
    while (is_recording_.data && goal->duration > 0 && !goal_handle->is_canceling())
    {
      auto now = std::chrono::system_clock::now();
      auto elapsed = now - video_start_time_;
      auto remaining = desired_video_duration_ - elapsed;

      feedback->time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
      feedback->time_remaining = std::chrono::duration_cast<std::chrono::seconds>(remaining).count();;
      feedback->n_frames = n_frames_;

      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    if (goal_handle->is_canceling())
    {
      RCLCPP_WARN(rclcpp::get_logger("VideoRecorder"), "Recording cancelled!");
      stopRecording();
      result->header.stamp = node_->now();
      result->header.frame_id = camera_frame_;
      result->success = false;
      result->path = video_result_path_;
      goal_handle->abort(result);
    }

    // return the result
    result->header.stamp = node_->now();
    result->header.frame_id = camera_frame_;
    result->success = true;
    result->path = video_result_path_;
    goal_handle->succeed(result);
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("VideoRecorder"), "Unable to start recording; node is already recording to %s", video_path_.c_str());
    result->header.stamp = node_->now();
    result->header.frame_id = camera_frame_;
    result->success = false;
    result->path = video_result_path_;
    goal_handle->succeed(result);
  }
}

/*!
 * Handler for the StopRecording service. Stops any video recording in process, even if
 * we have not yet reached the duration specified in the argument to StartRecordingHandler.
 */
void VideoRecorderNode::executeStopRecording(const std::shared_ptr<GoalHandleStopRecording> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<StopRecordingAction::Feedback>();
  auto result = std::make_shared<StopRecordingAction::Result>();

  if (is_recording_.data)
  {
    RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Stopping recording to to %s", video_path_.c_str());

    pthread_mutex_lock(&video_recording_lock_);
    stopRecording();
    pthread_mutex_unlock(&video_recording_lock_);

    // calculate the total recording time in seconds
    auto stop_time = std::chrono::system_clock::now();
    auto elapsed = stop_time - video_start_time_;
    unsigned long seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

    result->header.stamp = node_->now();
    result->header.frame_id = camera_frame_;
    result->success = true;
    result->path = video_result_path_;
    result->size = filesize(video_path_);
    result->duration = seconds;
    goal_handle->succeed(result);
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("VideoRecorder"), "Unable to stop recording; node is not recording");
    result->header.stamp = node_->now();
    result->header.frame_id = camera_frame_;
    result->success = false;
    result->path = "";
    result->size = 0;
    result->duration = 0;
    goal_handle->succeed(result);
  }
}

/*!
 * Handler for the SaveImage action. Delays the specified number of seconds and then flags that the next frame
 * should be captured and saved to a file
 */
void VideoRecorderNode::executeSaveImage(const std::shared_ptr<GoalHandleSaveImage> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Executing save image action goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<SaveImageAction::Feedback>();
  auto result = std::make_shared<SaveImageAction::Result>();

  if (!(status_.status & video_recorder_msgs::msg::Status::RUNNING))
  {
    RCLCPP_WARN(rclcpp::get_logger("VideoRecorder"), "Unable to save image; no data received from the camera yet");
    result->header.stamp = node_->now();
    result->success = false;
    result->path = "";
    goal_handle->abort(result);
  }
  else if (!capture_next_frame_)
  {
    // First figure out the full path of the png/bmp/jpg file we're saving
    std::stringstream image_path_ss;
    std::stringstream result_path_ss;
    image_path_ss << out_dir_;
    if (mount_path_.length() > 0)
      result_path_ss << mount_path_;
    else
      result_path_ss << out_dir_;
    if (goal->filename.length() == 0)
    {
      std::string default_filename = defaultFilename("png");
      image_path_ss << default_filename;
      result_path_ss << default_filename;
    }
    else
    {
      image_path_ss << goal->filename;
      result_path_ss << goal->filename;
      // if the user specified a file extension, try to use that if we know what it is
      // otherwise use .png
      if (!ends_with(goal->filename, ".bmp") &&
          !ends_with(goal->filename, ".jpg") &&
          !ends_with(goal->filename, ".jpeg") &&
          !ends_with(goal->filename, ".png")
          // TODO: any more common formats we want to be able to support?
      )
      {
        image_path_ss << ".png";
        result_path_ss << ".png";
      }
    }
    image_path_ss >> image_path_;
    result_path_ss >> image_result_path_;

    rclcpp::Rate r(1);
    unsigned long time_remaining = goal->delay;
    RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Saving image in %d seconds to %s", goal->delay, image_path_.c_str());

    for(unsigned long i=0; i<time_remaining; i++)
    {
      feedback->time_remaining = time_remaining-i;
      goal_handle->publish_feedback(feedback);
      status_.status |= video_recorder_msgs::msg::Status::PHOTO_TIMER;
      r.sleep();
    }
    image_saved_ = false;
    capture_next_frame_ = true;

    while (!image_saved_)
      r.sleep();

    result->header.stamp = node_->now();
    result->header.frame_id = camera_frame_;
    result->success = true;
    result->path = image_result_path_;
    goal_handle->succeed(result);
    status_.status &= ~video_recorder_msgs::msg::Status::PHOTO_TIMER;
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("VideoRecorder"), "Already queued to record the next frame");
    result->header.stamp = node_->now();
    result->header.frame_id = camera_frame_;
    result->success = false;
    result->path = image_result_path_;
    goal_handle->succeed(result);
  }
}

/*!
 * Callback for the zoom_level subscription. Sets the zoom_level_ variable
 * according to the provided argument
 */
void VideoRecorderNode::zoomLevelCallback(const std_msgs::msg::Float64::ConstSharedPtr zoom)
{
  zoom_level_ = zoom->data;
}

/*!
 * Subscription to the image topic we're responsible for capturing.  As long as the node is alive
 * we keep an open subscription, but we only process the frame if we're either recording or about to
 * save a still image.
 */
void VideoRecorderNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img)
{
  // clear the waiting flag, set the running flag, increment the frames received
  status_.status &= ~video_recorder_msgs::msg::Status::WAITING;
  status_.status |= video_recorder_msgs::msg::Status::RUNNING;
  status_.frames_received_last_second++;

  if (is_recording_.data || capture_next_frame_)
  {
    cv::UMat m;
    bool conversion_ok = image2mat(*img, m);

    // Just kick out early if the conversion fails
    if(!conversion_ok)
      return;

    processImage(m);
  }
}

/*!
 * Subscription to the compressed image topic, used when ~compressed is true.
 * We only bother processing the frame if we're going to write it to the video or image files
 */
void VideoRecorderNode::compressedImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr img)
{
  // clear the waiting flag, set the running flag, increment the frames received
  status_.status &= ~video_recorder_msgs::msg::Status::WAITING;
  status_.status |= video_recorder_msgs::msg::Status::RUNNING;
  status_.frames_received_last_second++;

  if (is_recording_.data || capture_next_frame_)
  {
    cv::Mat m = cv::imdecode(img->data, cv::IMREAD_UNCHANGED);
    cv::UMat um = m.getUMat(cv::ACCESS_READ);
    processImage(um);
  }
}

/*!
 * Backend for imageCallback and compressedImageCallback to reduce duplication
 */
void VideoRecorderNode::processImage(const cv::UMat &m)
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

void VideoRecorderNode::appendFrame(const cv::UMat &img)
{
  cv::UMat resized = cv::UMat::zeros(output_height_, output_width_, CV_8UC3);
  letterbox_or_pillarbox(img, resized);
  pthread_mutex_lock(&video_recording_lock_);
  *(vout_) << resized;
  n_frames_++;

  // check if we should stop recording
  auto now = std::chrono::system_clock::now();
  auto elapsed = now - video_start_time_;
  if (max_video_duration_ > std::chrono::seconds(0) && elapsed >= max_video_duration_)
  {
    RCLCPP_WARN(rclcpp::get_logger("VideoRecorder"), "Video recording reached hard time limit. Stopping recording %s", video_path_.c_str());
    stopRecording();
  }
  else if (desired_video_duration_ > std::chrono::seconds(0) && elapsed >= desired_video_duration_)
  {
    RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "User-specified duration elapsed; stopping recording %s", video_path_.c_str());
    stopRecording();
  }
  pthread_mutex_unlock(&video_recording_lock_);
}

/*!
 * Open the video file, create the cv::VideoWriter instance
 */
void VideoRecorderNode::startRecording()
{
  if (record_metadata_)
    recordMetadata(video_path_);

  // create the video_writer
  pthread_mutex_lock(&video_recording_lock_);
  // record the start time of the recording
  video_start_time_ = std::chrono::system_clock::now();
  n_frames_ = 0;
  vout_ = createVideoWriter();
  is_recording_.data = true;
  is_recording_publisher_->publish(is_recording_);  // update the latched topic when we start recording
  pthread_mutex_unlock(&video_recording_lock_);
}

/*!
 * Close the video file and destroy the cv::VideoWriter instance
 */
void VideoRecorderNode::stopRecording()
{
  is_recording_.data = false;
  is_recording_publisher_->publish(is_recording_);  // update the latched topic when we stop recording
  if (vout_ != NULL)
  {
    vout_->release();
    delete vout_;
    vout_ = NULL;
  }
}

void VideoRecorderNode::saveImage(const cv::UMat &img)
{
  capture_next_frame_ = false;
  cv::imwrite(image_path_, img);
  image_saved_ = true;
}

/*!
 * Converts the raw sensor image into a BGR8 cv::UMat object
 */
bool VideoRecorderNode::image2mat(const sensor_msgs::msg::Image &src, cv::UMat &dst)
{
  // realsense2_camera seems to have a bug where it uses the OpenCV encoding, which breaks the conversion
  // so make a shallow copy and ensure the encoding is correct
  sensor_msgs::msg::Image img_fixed;
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
      RCLCPP_WARN(rclcpp::get_logger("VideoRecorder"), "Unsupported image format %s", img_fixed.encoding.c_str());
      frame_ok = false;
    }
  }
  catch(cv_bridge::Exception &err)
  {
    RCLCPP_ERROR(rclcpp::get_logger("VideoRecorder"), "OpenCV Error %s", err.what());
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
  RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Recording metadata for %s...", filename.c_str());

  // get the current camera pose and robot position on the map
  geometry_msgs::msg::Twist pos_map = lookupTransform("map", "base_link");
  geometry_msgs::msg::Twist pos_cam = lookupTransform("base_link", camera_frame_);

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
       << "  \"topic\": \"" << img_topic_ << "\"," << std::endl
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
       << "  \"zoom\": " << zoom_lvl.c_str() << std::endl
       << "}" << std::endl;
  fout.close();

  RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Location on map when saving %s: %f %f %f %f %f %f",
    filename.c_str(), pos_map.linear.x, pos_map.linear.y, pos_map.linear.z,
    pos_map.angular.x, pos_map.angular.y, pos_map.angular.z);
  RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "Camera position when saving %s: %f %f %f %f %f %f",
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
geometry_msgs::msg::Twist VideoRecorderNode::lookupTransform(const std::string &target_frame, const std::string &fixed_frame)
{
  geometry_msgs::msg::Twist result;

  try
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock(), 2s);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
      target_frame, fixed_frame, rclcpp::Time(0), 5s);

    result.linear.x = tf_stamped.transform.translation.x;
    result.linear.y = tf_stamped.transform.translation.y;
    result.linear.z = tf_stamped.transform.translation.z;

    tf2::Quaternion rotq(tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z, tf_stamped.transform.rotation.w);
    tf2::Matrix3x3 m(rotq);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    result.angular.x = roll;
    result.angular.y = pitch;
    result.angular.z = yaw;
  }
  catch(tf2::LookupException err)
  {
    RCLCPP_WARN(rclcpp::get_logger("VideoRecorder"), "Failed to look up transform from %s to %s", fixed_frame.c_str(), target_frame.c_str());
    result.linear.x = 0;
    result.linear.y = 0;
    result.linear.z = 0;
    result.angular.x = 0;
    result.angular.y = 0;
    result.angular.z = 0;
  }

  return result;
}

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
void VideoRecorderNode::createOutputDirectory(const std::string &dir)
{
  if (fileExists(dir) && isDirectory(dir))
  {
    // everything is ok!
    RCLCPP_INFO(rclcpp::get_logger("VideoRecorder"), "%s is a valid output directory", dir.c_str());
  }
  else if (fileExists(dir) && !isDirectory(dir))
  {
    // dir is a file, not a directory; this is an error!
    RCLCPP_ERROR(rclcpp::get_logger("VideoRecorder"), "%s is a file not a directory!", dir.c_str());
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
        RCLCPP_ERROR(rclcpp::get_logger("VideoRecorder"), "%s is a file, not a directory!", substr.c_str());
        exit(1);
      }
      else if (!isDirectory(substr))
      {
        RCLCPP_WARN(rclcpp::get_logger("VideoRecorder"), "Creating %s", substr.c_str());
        int ret = mkdir(substr.c_str(), 0755);
        if (ret != 0)
        {
          RCLCPP_ERROR(rclcpp::get_logger("VideoRecorder"), "Failed to create directory %s: error %d", substr.c_str(), ret);
          exit(1);
        }
      }

      start_at = pos+1;
    }
  }
}

/*!
 * Yes, this is a gross C function, but std::filesystem isn't available in C++14
 * and Melodic doesn't support C++17
 */
unsigned long VideoRecorderNode::filesize(std::string path)
{
  struct stat buf;
  stat(path.c_str(), &buf);
  unsigned long size = buf.st_size;
  return size;
}

/*!
 * Shamelessly copied from https://stackoverflow.com/questions/874134/find-out-if-string-ends-with-another-string-in-c
 */
inline bool VideoRecorderNode::ends_with(std::string const & value, std::string const & ending)
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
void VideoRecorderNode::letterbox_or_pillarbox(const cv::UMat &src, cv::UMat &dst)
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

} //namespace video_recorder


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MAIN
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*!
 * Create the node and spin. Nothing fancy to see here.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("video_recorder_node");

  video_recorder::VideoRecorderNode video_recorder(node);
  rclcpp::spin(node);
  return 0;
}
