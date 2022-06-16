#include <cstdlib>
#include <ctime>
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

using namespace video_recorder;
using namespace video_recorder_msgs;

/*!
 * Yes, this is a gross C function, but std::filesystem isn't available in C++14
 * and Melodic doesn't support C++17
 */
static unsigned long filesize(std::string path)
{
  struct stat *buf = (struct stat*)malloc(sizeof(struct stat));
  stat(path.c_str(), buf);
  unsigned long size = buf->st_size;
  free(buf);
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

/*!
 * Creates the start/stop services, loads the ROS parameters, subscribes to the input topic
 */
VideoRecorderNode::VideoRecorderNode(ros::NodeHandle &nh) :
  nh_(nh),
  frame_service_(nh, "save_image", boost::bind(&VideoRecorderNode::saveImageHandler, this, _1), false),
  start_service_(nh, "start_recording", boost::bind(&VideoRecorderNode::startRecordingHandler, this, _1), false),
  stop_service_(nh, "stop_recording", boost::bind(&VideoRecorderNode::stopRecordingHandler, this, _1), false)
{
  is_recording_ = std_msgs::Bool();
  is_recording_.data = false;
  capture_next_frame_ = false;
  vout_ = NULL;

  pthread_mutex_init(&video_recording_lock_, NULL);

  // the topic we subscribe to is defined as a parameter
  loadParams();
  is_recording_pub_ = nh.advertise<std_msgs::Bool>("is_recording", 1);
  img_sub_ = nh.subscribe(img_topic_, 1, &VideoRecorderNode::imageCallback, this);

  frame_service_.start();
  start_service_.start();
  stop_service_.start();
}

/*!
 * Stops recording if we are currently saving a video and closes any open files
 */
VideoRecorderNode::~VideoRecorderNode()
{
  stopRecording();
}

/*!
 * Loads the necessary ROS parameters needed to finish constructing the node
 */
void VideoRecorderNode::loadParams()
{
  nh_.param<std::string>("topic", img_topic_, "/camera/image_raw");
  nh_.param<std::string>("out_dir", out_dir_, "/tmp");
  nh_.param<double>("fps", fps_, 30.0);
  nh_.param<int>("output_height", output_height_, 480);
  nh_.param<int>("output_width", output_width_, 640);

  if (out_dir_[out_dir_.length()-1] != '/')
  {
    out_dir_.push_back('/');
  }
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
  if (!is_recording_.data)
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

    // create the video_writer
    ROS_INFO("Recording to %s for %d seconds (0=inf)", video_path_.c_str(), (int)goal->duration);
    pthread_mutex_lock(&video_recording_lock_);
    // record the start time of the recording and max duration
    max_video_duration_ = std::chrono::seconds(goal->duration);
    video_start_time_ = std::chrono::system_clock::now();
    n_frames_ = 0;
    vout_ = createVideoWriter();
    is_recording_.data = true;
    pthread_mutex_unlock(&video_recording_lock_);

    // publish feedback while we're recording if we specified a duration
    ros::Rate rate(10);
    video_recorder_msgs::StartRecordingFeedback feedback;
    while (is_recording_.data && goal->duration > 0)
    {
      auto now = std::chrono::system_clock::now();
      auto elapsed = now - video_start_time_;
      auto remaining = max_video_duration_ - elapsed;

      feedback.time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
      feedback.time_remaining = std::chrono::duration_cast<std::chrono::seconds>(remaining).count();;
      feedback.n_frames = n_frames_;

      start_service_.publishFeedback(feedback);
      rate.sleep();
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
  if (!capture_next_frame_)
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
      r.sleep();
    }
    image_saved_ = false;
    capture_next_frame_ = true;

    while (!image_saved_)
      r.sleep();

    result.path = image_path_;
    result.success = true;
    frame_service_.setSucceeded(result);
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
 * Subscription to the image topic we're responsible for capturing.  As long as the node is alive
 * we keep an open subscription, but we only process the frame if we're either recording or about to
 * save a still image.
 */
void VideoRecorderNode::imageCallback(const sensor_msgs::Image &img)
{
  if (is_recording_.data || capture_next_frame_)
  {
    cv::Mat m;
    bool conversion_ok = image2mat(img, m);

    // Just kick out early if the conversion fails
    if(!conversion_ok)
      return;

    if (is_recording_.data)
    {
      appendFrame(m);
    }

    if (capture_next_frame_)
    {
      saveImage(m);
    }
  }

  is_recording_pub_.publish(is_recording_);
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
  if (max_video_duration_ > std::chrono::seconds(0))
  {
    auto now = std::chrono::system_clock::now();
    auto elapsed = now - video_start_time_;
    if (elapsed >= max_video_duration_)
    {
      ROS_INFO("User-specified duration elapsed; stopping recording %s", video_path_.c_str());

      stopRecording();
    }
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
 * Create the node and spin. Nothing fancy to see here.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_recorder_node");
  ros::NodeHandle nh("~");
  VideoRecorderNode node(nh);
  ros::spin();
  return 0;
}
