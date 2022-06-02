#include <chrono>
#include <string>

#include <opencv2/videoio.hpp>

#include <pthread.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <video_recorder/SaveImage.h>
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

    const bool isRecording(){ return is_recording_.data; }

  private:
    // Node handle, subscriptions, publications, services
    ros::NodeHandle &nh_;
    ros::ServiceServer frame_service_;
    ros::ServiceServer start_service_;
    ros::ServiceServer stop_service_;
    ros::Subscriber img_sub_;
    ros::Publisher is_recording_pub_;

    // ROS parameters
    void loadParams();
    std::string img_topic_;
    std::string out_dir_;
    double fps_;

    // Thread control
    pthread_mutex_t video_recording_lock_;

    // Service & subscription callbacks
    bool saveImageHandler(video_recorder::SaveImage::Request &req, video_recorder::SaveImage::Response &res);
    bool startRecordingHandler(video_recorder::StartRecording::Request &req, video_recorder::StartRecording::Response &res);
    bool stopRecordingHandler(video_recorder::StopRecording::Request &req, video_recorder::StopRecording::Response &res);
    void imageCallback(const sensor_msgs::Image &img);

    // Video capture
    std_msgs::Bool is_recording_;
    std::chrono::duration<unsigned long, std::ratio<1> > max_video_duration_;
    std::chrono::time_point<std::chrono::system_clock> video_start_time_;
    std::string video_path_;
    cv::VideoWriter *vout_;
    cv::VideoWriter *createVideoWriter(const int width, const int height);
    void appendFrame(const cv::Mat &img);
    void stopRecording();

    // Still image capture
    bool capture_next_frame_;
    std::string image_path_;
    void saveImage(const cv::Mat &img);

    // General Utilities
    bool image2mat(const sensor_msgs::Image &src, cv::Mat &dst);
    std::string defaultFilename(std::string extension);
  };
}
