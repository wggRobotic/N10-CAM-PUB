#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <future>
#include <mutex>
#include <unordered_map>

using namespace std::chrono_literals;

class CamPublisher : public rclcpp::Node
{
public:
  CamPublisher() : Node("cam_publisher"), stop_thread_(false)
  {}

  ~CamPublisher()
  {
    stop_thread_ = true;
    if (capture_thread_.joinable())
    {
      capture_thread_.join();
    }
  }

  void init(image_transport::ImageTransport &it_)
  {
    // Declare parameters
    this->declare_parameter<std::string>("topic_name", "/n10/rear/color");
    this->declare_parameter<std::string>("camera_name", "video0");
    this->declare_parameter<int>("framerate", 30);      // Default framerate
    this->declare_parameter<std::string>("resolution", "720p");  // Default resolution label

    // Get parameters
    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("camera_name", camera_name_);
    this->get_parameter("framerate", framerate_);
    this->get_parameter("resolution", resolution_label_);

    // Define resolution mappings
    std::unordered_map<std::string, std::pair<int, int>> resolution_map = {
        {"480p", {640, 480}},
        {"720p", {1280, 720}},
        {"1080p", {1920, 1080}},
        {"4K", {3840, 2160}}  // Add more resolutions if needed
    };

    // Get width and height from resolution label
    auto it = resolution_map.find(resolution_label_);
    if (it != resolution_map.end())
    {
      width_ = it->second.first;
      height_ = it->second.second;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Invalid resolution specified. Using default 720p.");
      width_ = 1280;
      height_ = 720; // Default to 720p if an unknown resolution label is provided
    }

    // Initialize publisher
    pub_ = it_.advertise(topic_name_, 1);

    // Initialize video capture
    std::string camera_path = "/dev/" + camera_name_;
    cap_.open(camera_path);
    if (!cap_.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not open camera with index %s", camera_path.c_str());
      rclcpp::shutdown();
    }

    // Set resolution and framerate
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, framerate_);

    // Start async capture thread
    capture_thread_ = std::thread(&CamPublisher::capture_loop, this);
  }

private:
  void capture_loop()
  {
    auto last_published_time = std::chrono::steady_clock::now();  // Track the last published time
    auto frame_duration = std::chrono::milliseconds(1000 / framerate_);  // Frame interval based on framerate

    while (!stop_thread_)
    {
      cv::Mat frame;
      {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        cap_ >> frame;  // Capture as fast as possible
      }

      if (!frame.empty())
      {
        auto now = std::chrono::steady_clock::now();
        // Only publish if enough time has passed since the last published frame
        if (now - last_published_time >= frame_duration)
        {
          auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
          {
            std::lock_guard<std::mutex> lock(pub_mutex_);
            pub_.publish(msg);
          }
          last_published_time = now;  // Update the last published time
        }
      }

      // To avoid busy-waiting, sleep for a short duration
      std::this_thread::sleep_for(5ms);  // Adjust as needed to avoid excessive CPU usage
    }
  }



  image_transport::Publisher pub_;
  cv::VideoCapture cap_;
  std::thread capture_thread_;
  std::atomic<bool> stop_thread_;
  std::mutex cap_mutex_;
  std::mutex pub_mutex_;
  std::string topic_name_;
  std::string camera_name_;
  std::string resolution_label_;
  int framerate_;
  int width_;
  int height_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CamPublisher>();
  image_transport::ImageTransport it_(node);
  node->init(it_);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
