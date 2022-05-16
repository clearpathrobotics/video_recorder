#include <string>

#include <opencv2/videoio.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <video_recorder/StartRecording.h>
#include <video_recorder/StopRecording.h>

namespace video_recorder
{
  /*!
   * A node that subscribes to a sensor_msgs/Image topic and records video to a file
   * Topic and output directory are specified as rosparams
   * File name of the recorded file (minus extension) is specified by calling the StartRecording service
   */
  class VideoRecorderNode
  {
  public:
    VideoRecorderNode(ros::NodeHandle &nh);
    ~VideoRecorderNode();

    const bool isRecording(){ return is_recording_; }

  private:
    ros::NodeHandle &nh_;
    ros::ServiceServer start_service_;
    ros::ServiceServer stop_service_;
    ros::Subscriber img_sub_;

    bool is_recording_;

    std::string img_topic_;
    std::string encoding_;
    std::string out_dir_;
    std::string out_path_;
    double fps_;

    cv::VideoWriter *vout_;

    void loadParams();

    bool startRecording(video_recorder::StartRecording::Request &req, video_recorder::StartRecording::Response &res);
    bool stopRecording(video_recorder::StopRecording::Request &req, video_recorder::StopRecording::Response &res);

    void imageCallback(const sensor_msgs::Image &img);
  };
}
