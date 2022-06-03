#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sys/stat.h>

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
 * Creates the start/stop services, loads the ROS parameters, subscribes to the input topic
 */
VideoRecorderNode::VideoRecorderNode(ros::NodeHandle &nh) :
  nh_(nh),
  vout_(NULL),
  capture_next_frame_(false)
{
  is_recording_ = std_msgs::Bool();
  is_recording_.data = false;

  pthread_mutex_init(&video_recording_lock_, NULL);

  // the topic we subscribe to is defined as a parameter
  loadParams();
  frame_service_ = nh.advertiseService(img_topic_ + "/save_image", &VideoRecorderNode::saveImageHandler, this);
  start_service_ = nh.advertiseService(img_topic_ + "/start_recording", &VideoRecorderNode::startRecordingHandler, this);
  stop_service_ = nh.advertiseService(img_topic_ + "/stop_recording", &VideoRecorderNode::stopRecordingHandler, this);
  is_recording_pub_ = nh.advertise<std_msgs::Bool>(img_topic_ + "/is_recording", 1);
  img_sub_ = nh.subscribe(img_topic_, 1, &VideoRecorderNode::imageCallback, this);
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
 */
bool VideoRecorderNode::startRecordingHandler(
  StartRecording::Request &req,
  StartRecording::Response &res)
{
  // Because initializing the video recording is a relatively lengthy operation
  // it's theoretically possible for the user to invoke the service multiple times before we're ready.
  // Therefore use a mutex to ensure we only create one file at a time; otherwise we might introduce
  // a memory leak.
  pthread_mutex_lock(&video_recording_lock_);

  bool ok = true;
  if (!is_recording_.data)
  {
    // First figure out the full path of the .avi file we're saving
    std::stringstream ss;
    ss << out_dir_;
    if (req.filename.length() == 0)
    {
      ss << defaultFilename("avi");
    }
    else
    {
      ss << req.filename;
      if (!ends_with(req.filename, ".avi"))
      {
        ss << ".avi";
      }
    }
    ss >> video_path_;

    // record the start time of the recording and max duration
    max_video_duration_ = std::chrono::seconds(req.duration);
    video_start_time_ = std::chrono::system_clock::now();

    // we're ready! signal that we're recording so the image subscriber can start recording frames
    is_recording_.data = true;
    ROS_INFO("Recoring to %s", video_path_.c_str());
    res.path = video_path_;
  }
  else
  {
    ROS_WARN("Unable to start recording; node is already recording to %s", video_path_.c_str());
    res.path = "";
    ok = false;
  }

  pthread_mutex_unlock(&video_recording_lock_);
  return ok;
}

/*!
 * Handler for the StopRecording service. Stops any video recording in process, even if
 * we have not yet reached the duration specified in the argument to StartRecordingHandler.
 * Returns an error if there is no recording to stop.
 */
bool VideoRecorderNode::stopRecordingHandler(
  StopRecording::Request &req,
  StopRecording::Response &res)
{
  pthread_mutex_lock(&video_recording_lock_);

  if (is_recording_.data)
  {
    stopRecording();

    // calculate the total recording time in seconds
    auto stop_time = std::chrono::system_clock::now();
    auto elapsed = stop_time - video_start_time_;
    unsigned long seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

    res.duration = seconds;
    res.path = video_path_;
    res.size = filesize(video_path_);
  }
  else
  {
    ROS_WARN("Unable to stop recording; node is not recording yet");
    res.path = "";
    res.size = 0;
  }

  pthread_mutex_unlock(&video_recording_lock_);
  return res.size > 0;
}

/*!
 * Handler for the SaveImage service. Signals that the next frame received by the subscription
 * shall be saved to a file
 * Returns an error if we're already queued to record an image but haven't actually saved it yet
 */
bool VideoRecorderNode::saveImageHandler(
  SaveImage::Request &req,
  SaveImage::Response &res)
{
  bool ok = true;
  if (!capture_next_frame_)
  {
    // First figure out the full path of the .avi file we're saving
    std::stringstream ss;
    ss << out_dir_;
    if (req.filename.length() == 0)
    {
      ss << defaultFilename("png");
    }
    else
    {
      ss << req.filename;
      // if the user specified a file extension, try to use that if we know what it is
      // otherwise use .png
      if (!ends_with(req.filename, ".bmp") &&
          !ends_with(req.filename, ".jpg") &&
          !ends_with(req.filename, ".jpeg") &&
          !ends_with(req.filename, ".png")
          // TODO: any more common formats we want to be able to support?
      )
      {
        ss << ".png";
      }
    }
    ss >> image_path_;

    capture_next_frame_ = true;
    ROS_INFO("Saving next frame to to %s", image_path_.c_str());
    res.path = image_path_;
  }
  else
  {
    ROS_WARN("Already queued to record the next frame");
    ok = false;
    res.path = "";
  }
  return ok;
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
cv::VideoWriter *VideoRecorderNode::createVideoWriter(const int width, const int height)
{
  return new cv::VideoWriter(video_path_, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps_, cv::Size(width, height), true);
}

void VideoRecorderNode::appendFrame(const cv::Mat &img)
{
  // create the cv::VideoWriter instance if we need to
  if (vout_ == NULL)
  {
    vout_ = createVideoWriter(img.cols, img.rows);
  }

  *(vout_) << img;

  // check if we should stop recording
  if (max_video_duration_ > std::chrono::seconds(0))
  {
    auto now = std::chrono::system_clock::now();
    auto elapsed = now - video_start_time_;
    if (elapsed >= max_video_duration_)
    {
      ROS_INFO("User-specified duration elapsed; stopping recording %s", video_path_.c_str());
      pthread_mutex_lock(&video_recording_lock_);
      stopRecording();
      pthread_mutex_unlock(&video_recording_lock_);
    }
  }
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
      cv_ptr->image.convertTo(dst, CV_8UC3);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::RGBA16)
    {
      cv::cvtColor(cv_ptr->image,dst, cv::COLOR_RGB2BGR);
      cv_ptr->image.convertTo(dst, CV_8UC3);
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
      cv_ptr->image.convertTo(dst, CV_8UC3);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::MONO8)
    {
      cv::cvtColor(cv_ptr->image, dst, cv::COLOR_GRAY2BGR);
    }
    else if (img_fixed.encoding == sensor_msgs::image_encodings::MONO16)
    {
      cv::cvtColor(cv_ptr->image, dst, cv::COLOR_GRAY2BGR);
      cv_ptr->image.convertTo(dst, CV_8UC3);
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
