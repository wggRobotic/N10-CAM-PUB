#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <future>
#include <mutex>

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

    // Get parameters
    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("camera_name", camera_name_);

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

    // Start async capture thread
    capture_thread_ = std::thread(&CamPublisher::capture_loop, this);
  }

private:
  void capture_loop()
  {
    while (!stop_thread_)
    {
      cv::Mat frame;
      {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        cap_ >> frame;
      }

      if (!frame.empty())
      {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        {
          std::lock_guard<std::mutex> lock(pub_mutex_);
          pub_.publish(msg);
        }
      }

      std::this_thread::sleep_for(33ms); // Adjust the sleep duration as needed
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
