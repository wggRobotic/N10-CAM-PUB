#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class CamPublisher : public rclcpp::Node
{
public:
  CamPublisher() : Node("cam_publisher"){};

  void init(image_transport::ImageTransport &it_){
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

    // Create a timer to publish images
    timer_ = this->create_wall_timer(
        15ms, std::bind(&CamPublisher::publish_image, this));
  }
private:
  void publish_image()
  {
    cv::Mat frame;
    cap_ >> frame;

    if (!frame.empty())
    {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      pub_.publish(msg);
      cv::waitKey(1);
    }
  }

  image_transport::Publisher pub_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string topic_name_;
  std::string camera_name_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CamPublisher>();
  image_transport::ImageTransport it(node);
  node->init(it);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
