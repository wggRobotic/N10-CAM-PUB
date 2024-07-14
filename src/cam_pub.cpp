#include <chrono>
#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("n10_cam_pub", options);
  
  // Declare and get parameters
  node->declare_parameter<std::string>("topic_name", "/n10/rear/color");
  node->declare_parameter<std::string>("camera_name", "video0");
  std::string topic_name = node->get_parameter("topic_name").as_string();
  std::string camera_name = node->get_parameter("camera_name").as_string();

  // Initialize publisher
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise(topic_name, 1);

  // Open the video capture
  std::string camera_path = "/dev/" + camera_name;
  cv::VideoCapture cap(camera_path);
  if (!cap.isOpened())
  {
    RCLCPP_ERROR(node->get_logger(), "Could not open camera: %s", camera_path.c_str());
    return 1;
  }

  cv::Mat frame;
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;

  rclcpp::WallRate loop_rate(5ms);
  while (rclcpp::ok())
  {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if (!frame.empty())
    {
      msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
