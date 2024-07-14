#include <chrono>
#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>  // Correct header file for Image message

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("n10_cam_pub", options);
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("/n10/rear/color", 1);

  cv::VideoCapture cap(0);
  if (!cap.isOpened()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open camera");
    return 1;
  }

  cv::Mat frame;
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;

  rclcpp::WallRate loop_rate(33ms);  // Approx. 30 FPS
  while (rclcpp::ok())
  {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if (!frame.empty())
    {
      hdr.stamp = node->now();
      msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
