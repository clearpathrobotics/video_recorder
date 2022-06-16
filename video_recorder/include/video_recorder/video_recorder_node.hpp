#include <chrono>
#include <string>

#include <opencv2/videoio.hpp>

#include <pthread.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <video_recorder_msgs/SaveImageAction.h>
#include <video_recorder_msgs/StartRecordingAction.h>
#include <video_recorder_msgs/StopRecordingAction.h>

typedef actionlib::SimpleActionServer<video_recorder_msgs::SaveImageAction> SaveImageActionServer;
typedef actionlib::SimpleActionServer<video_recorder_msgs::StartRecordingAction> StartRecordingActionServer;
typedef actionlib::SimpleActionServer<video_recorder_msgs::StopRecordingAction> StopRecordingActionServer;

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
    SaveImageActionServer frame_service_;
    StartRecordingActionServer start_service_;
    StopRecordingActionServer stop_service_;
    ros::Subscriber img_sub_;
    ros::Publisher is_recording_pub_;

    // ROS parameters
    void loadParams();
    std::string img_topic_;
    std::string out_dir_;
    double fps_;
    int output_height_;
    int output_width_;
    bool compressed_;

    // Thread control
    pthread_mutex_t video_recording_lock_;

    // Service & subscription callbacks
    void saveImageHandler(const video_recorder_msgs::SaveImageGoalConstPtr& goal);
    void startRecordingHandler(const video_recorder_msgs::StartRecordingGoalConstPtr& goal);
    void stopRecordingHandler(const video_recorder_msgs::StopRecordingGoalConstPtr& goal);
    void imageCallback(const sensor_msgs::Image &img);
    void compressedImageCallback(const sensor_msgs::CompressedImage &img);

    // Video capture
    std_msgs::Bool is_recording_;
    unsigned long n_frames_;
    std::chrono::duration<unsigned long, std::ratio<1> > max_video_duration_;
    std::chrono::time_point<std::chrono::system_clock> video_start_time_;
    std::string video_path_;
    cv::VideoWriter *vout_;
    cv::VideoWriter *createVideoWriter();
    void appendFrame(const cv::Mat &img);
    void stopRecording();

    // Still image capture
    bool capture_next_frame_;
    bool image_saved_;
    std::string image_path_;
    void saveImage(const cv::Mat &img);

    // General Utilities
    bool image2mat(const sensor_msgs::Image &src, cv::Mat &dst);
    std::string defaultFilename(std::string extension);
  };
}
