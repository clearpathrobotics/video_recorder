#include <chrono>
#include <string>

#include <opencv2/videoio.hpp>

#include <pthread.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <video_recorder_msgs/SaveImageAction.h>
#include <video_recorder_msgs/StartRecordingAction.h>
#include <video_recorder_msgs/StopRecordingAction.h>
#include <video_recorder_msgs/Status.h>

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
    VideoRecorderNode(ros::NodeHandle &nh,
      const std::string &img_topic,
      const std::string &out_dir,
      const std::string &mount_path,
      const std::string &camera_frame,
      const double fps,
      const double output_height,
      const double output_width,
      const bool compressed,
      const bool record_metadata,
      const bool supports_zoom);
    ~VideoRecorderNode();

    const bool isRecording(){ return is_recording_.data; }

  private:
    // Node handle, subscriptions, publications, services
    ros::NodeHandle &nh_;
    SaveImageActionServer frame_service_;
    StartRecordingActionServer start_service_;
    StopRecordingActionServer stop_service_;
    ros::Subscriber img_sub_;
    ros::Subscriber zoom_sub_;
    ros::Publisher is_recording_pub_;

    // ROS parameters
    std::string img_topic_;
    std::string out_dir_;
    std::string mount_path_;
    std::string camera_frame_;
    double fps_;
    int output_height_;
    int output_width_;
    bool compressed_;
    bool record_metadata_;
    bool supports_zoom_;

    // Thread control
    pthread_mutex_t video_recording_lock_;

    // 1Hz background thread that reports the node's status
    pthread_t status_thread_;
    ros::Publisher status_pub_;
    video_recorder_msgs::Status status_;
    static void *statusPublisher(void *arg);


    // Service & subscription callbacks
    void saveImageHandler(const video_recorder_msgs::SaveImageGoalConstPtr& goal);
    void startRecordingHandler(const video_recorder_msgs::StartRecordingGoalConstPtr& goal);
    void stopRecordingHandler(const video_recorder_msgs::StopRecordingGoalConstPtr& goal);
    void imageCallback(const sensor_msgs::Image &img);
    void compressedImageCallback(const sensor_msgs::CompressedImage &img);
    void processImage(const cv::UMat &m);
    void processImage(const cv::Mat &m);
    void zoomLevelCallback(const std_msgs::Float64 &zoom);


    // Video capture
    std_msgs::Bool is_recording_;
    unsigned long n_frames_;
    std::chrono::duration<unsigned long, std::ratio<1> > desired_video_duration_;
    std::chrono::duration<unsigned long, std::ratio<1> > max_video_duration_;
    std::chrono::time_point<std::chrono::system_clock> video_start_time_;
    std::string video_path_;
    std::string video_result_path_;
    cv::VideoWriter *vout_;
    cv::VideoWriter *createVideoWriter();
    void appendFrame(const cv::UMat &img);
    void startRecording();
    void stopRecording();

    // Still image capture
    bool capture_next_frame_;
    bool image_saved_;
    std::string image_path_;
    std::string image_result_path_;
    void saveImage(const cv::UMat &img);

    // General Utilities
    bool image2mat(const sensor_msgs::Image &src, cv::UMat &dst);
    std::string defaultFilename(std::string extension);

    // Meta-data
    // Robot's current joint states, position on the map, etc...
    double zoom_level_;
    void recordMetadata(const std::string &filename);
    geometry_msgs::Twist lookupTransform(const std::string &target_frame, const std::string &fixed_frame);
  };
}
