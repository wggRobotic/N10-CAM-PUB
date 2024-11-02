#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <iostream>

class QRCodeScannerNode : public rclcpp::Node, public std::enable_shared_from_this<QRCodeScannerNode> {
public:
    QRCodeScannerNode() : Node("qr_code_scanner") {
        // Declare parameter for the topic name
        this->declare_parameter<std::string>("topic_name", "/n10/rear/color");

        // Get the topic name parameter
        this->get_parameter("topic_name", topic_name_);

        

        RCLCPP_INFO(this->get_logger(), "QR Code Scanner Node started");
        RCLCPP_INFO(this->get_logger(), "Subscribed to image topic (via image_transport): %s", topic_name_.c_str());
    }

    void init(image_transport::ImageTransport &it_)
    {
        // Initialize QRCodeDetector
        qrDecoder_ = cv::QRCodeDetector();

        // Initialize image_transport with the shared pointer
        image_subscription_ = it_.subscribe(
            topic_name_,
            10,
            std::bind(&QRCodeScannerNode::imageCallback, this, std::placeholders::_1)
        );
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        // Convert ROS image message to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Decode QR code from the image
        std::string decodedText = qrDecoder_.detectAndDecode(cv_ptr->image);

        if (decodedText.empty()) {
            RCLCPP_INFO(this->get_logger(), "No QR code detected.");
        } else {
            RCLCPP_INFO(this->get_logger(), "QR Code Content: %s", decodedText.c_str());
        }
    }
    

    cv::QRCodeDetector qrDecoder_;
    image_transport::Subscriber image_subscription_;
    std::string topic_name_;  // To store the topic name parameter
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<QRCodeScannerNode>();
    image_transport::ImageTransport it_(node);
    node->init(it_);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
