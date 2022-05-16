#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sys/stat.h>

#include <video_recorder/video_recorder_node.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace video_recorder;

/*!
 * Yes, this is a gross C function, but std::filessystem isn't available in C++14
 * and Melodic doesn't support C++17
 */
static int filesize(std::string path)
{
  struct stat *buf = (struct stat*)malloc(sizeof(struct stat));
  stat(path.c_str(), buf);
  int size = buf->st_size;
  free(buf);
  return size;
}

/*!
 * Creates the start/stop services, loads the ROS parameters, subscribes to the input topic
 */
VideoRecorderNode::VideoRecorderNode(ros::NodeHandle &nh) : nh_(nh), is_recording_(false), vout_(NULL)
{
  start_service_ = nh.advertiseService("start_recording", &VideoRecorderNode::startRecording, this);
  stop_service_ = nh.advertiseService("stop_recording", &VideoRecorderNode::stopRecording, this);

  loadParams();

  ROS_WARN("Subscribing to %s", img_topic_.c_str());
  img_sub_ = nh.subscribe(img_topic_, 1, &VideoRecorderNode::imageCallback, this);
}

VideoRecorderNode::~VideoRecorderNode()
{
  if (vout_ != NULL)
  {
    if (vout_->isOpened())
      vout_->release();

    delete vout_;
  }
}

void VideoRecorderNode::loadParams()
{
  nh_.param<std::string>("topic", img_topic_, "/camera/image_raw");
  nh_.param<std::string>("out_dir", out_dir_, getenv("HOME"));
  nh_.param<std::string>("encoding", encoding_, "bgr8");
  nh_.param<double>("fps", fps_, 30.0);

  if (out_dir_[out_dir_.length()-1] != '/')
  {
    out_dir_.push_back('/');
  }
}

bool VideoRecorderNode::startRecording(
  video_recorder::StartRecording::Request &req,
  video_recorder::StartRecording::Response &res)
{
  if (!is_recording_)
  {
    is_recording_ = true;

    std::stringstream ss;
    ss << out_dir_;
    if (req.filename.length() == 0)
    {
      // if the user didn't give us a filename use the current time as the default
      char time_str [80];
      std::time_t rawtime;
      std::tm* timeinfo;
      std::time(&rawtime);
      timeinfo = std::localtime(&rawtime);
      std::strftime(time_str,80,"%Y%m%d%H%M%S",timeinfo);
      std::puts(time_str);
      ss << time_str;
    }
    else
    {
      // otherwise use the provided filename
      ss << req.filename;
    }

    // add the extension
    ss << ".avi";

    ss >> out_path_;
    ROS_INFO("Recoring to %s", out_path_.c_str());

    res.success = true;
  }
  else
  {
    ROS_WARN("Unable to start recording; node is already recording to %s", out_path_.c_str());
    res.success = false;
  }

  return res.success;
}

bool VideoRecorderNode::stopRecording(
  video_recorder::StopRecording::Request &req,
  video_recorder::StopRecording::Response &res)
{
  if (is_recording_)
  {
    is_recording_ = false;
    vout_->release();
    delete vout_;
    vout_ = NULL;

    res.path = out_path_;
    res.size = filesize(out_path_);

    cv::destroyWindow(out_path_.c_str());
  }
  else
  {
    ROS_WARN("Unable to stop recording; node is not recording yet");
    res.path = "";
    res.size = 0;
  }

  return res.size > 0;
}

void VideoRecorderNode::imageCallback(const sensor_msgs::Image &img)
{
  if (is_recording_)
  {
    if (vout_ == NULL)
    {
      vout_ = new cv::VideoWriter(out_path_, CV_FOURCC('X', 'V', 'I', 'D'), fps_, cv::Size(img.width, img.height), true);
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, encoding_);
      (*vout_) << cv_ptr->image;

      cv::imshow(out_path_.c_str(), cv_ptr->image);
      cv::waitKey(1);
    }
    catch(cv_bridge::Exception &err)
    {
      ROS_ERROR("OpenCV Error %s", err.what());
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_recorder_node");
  ros::NodeHandle nh("~");
  VideoRecorderNode node(nh);
  ros::spin();
  return 0;
}
