#include <chrono>
#include <string>

#include <opencv2/videoio.hpp>

#include <pthread.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <video_recorder_msgs/action/save_image.hpp>
#include <video_recorder_msgs/action/start_recording.hpp>
#include <video_recorder_msgs/action/stop_recording.hpp>
#include <video_recorder_msgs/msg/status.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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
    typedef typename video_recorder_msgs::action::SaveImage SaveImageAction;
    typedef typename video_recorder_msgs::action::StartRecording StartRecordingAction;
    typedef typename video_recorder_msgs::action::StopRecording StopRecordingAction;

    typedef typename rclcpp_action::ServerGoalHandle<SaveImageAction> GoalHandleSaveImage;
    typedef typename rclcpp_action::ServerGoalHandle<StartRecordingAction> GoalHandleStartRecording;
    typedef typename rclcpp_action::ServerGoalHandle<StopRecordingAction> GoalHandleStopRecording;

    typedef typename rclcpp_action::Server<SaveImageAction>::SharedPtr SaveImageActionServer;
    typedef typename rclcpp_action::Server<StartRecordingAction>::SharedPtr StartRecordingActionServer;
    typedef typename rclcpp_action::Server<StopRecordingAction>::SharedPtr StopRecordingActionServer;

    VideoRecorderNode(std::shared_ptr<rclcpp::Node> node);

    ~VideoRecorderNode();

    const bool isRecording(){ return is_recording_.data; }

  private:
    // Node handle, subscriptions, publications, services
    std::shared_ptr<rclcpp::Node> node_ ;
    SaveImageActionServer frame_server_;
    StartRecordingActionServer start_server_;
    StopRecordingActionServer stop_server_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr img_compressed_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr zoom_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_recording_publisher_;

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
    rclcpp::Publisher<video_recorder_msgs::msg::Status>::SharedPtr status_publisher_;
    video_recorder_msgs::msg::Status status_;
    static void *statusPublisher(void *arg);


    // Service & subscription callbacks
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img);
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr img);
    void processImage(const cv::UMat &m);
    void processImage(const cv::Mat &m);
    void zoomLevelCallback(const std_msgs::msg::Float64::ConstSharedPtr zoom);

    void executeSaveImage(const std::shared_ptr<GoalHandleSaveImage> goal_handle);
    void executeStartRecording(const std::shared_ptr<GoalHandleStartRecording> goal_handle);
    void executeStopRecording(const std::shared_ptr<GoalHandleStopRecording> goal_handle);

    // Video capture
    std_msgs::msg::Bool is_recording_;
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
    bool image2mat(const sensor_msgs::msg::Image &src, cv::UMat &dst);
    std::string defaultFilename(std::string extension);

    // Meta-data
    // Robot's current joint states, position on the map, etc...
    double zoom_level_;
    void recordMetadata(const std::string &filename);
    geometry_msgs::msg::Twist lookupTransform(const std::string &target_frame, const std::string &fixed_frame);
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    // helper functions
    void createOutputDirectory(const std::string &dir);
    unsigned long filesize(std::string path);
    inline bool ends_with(std::string const & value, std::string const & ending);
    void letterbox_or_pillarbox(const cv::UMat &src, cv::UMat &dst);
  };
}
