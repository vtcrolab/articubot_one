#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class ColorDetectionNode : public rclcpp::Node {
public:
    ColorDetectionNode() : Node("color_detection") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&ColorDetectionNode::image_callback, this, std::placeholders::_1)
        );
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge conversion error: %s", e.what());
            return;
        }

        cv::Mat hsv;
        cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

        cv::Scalar yellow_lower(20, 100, 100), yellow_upper(30, 255, 255);
        cv::Scalar blue_lower(100, 150, 50), blue_upper(140, 255, 255);
        cv::Scalar red_lower1(0, 120, 70), red_upper1(10, 255, 255);
        cv::Scalar red_lower2(170, 120, 70), red_upper2(180, 255, 255);

        cv::Mat mask_yellow, mask_blue, mask_red1, mask_red2, mask_red;
        cv::inRange(hsv, yellow_lower, yellow_upper, mask_yellow);
        cv::inRange(hsv, blue_lower, blue_upper, mask_blue);
        cv::inRange(hsv, red_lower1, red_upper1, mask_red1);
        cv::inRange(hsv, red_lower2, red_upper2, mask_red2);
        mask_red = mask_red1 + mask_red2;

        std::vector<std::vector<cv::Point>> yellow_contours, blue_contours, red_contours;
        cv::findContours(mask_yellow, yellow_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(mask_blue, blue_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(mask_red, red_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        geometry_msgs::msg::Twist twist;

        // Debugging: Log the number of contours detected
        RCLCPP_INFO(this->get_logger(), "Yellow Contours: %ld", yellow_contours.size());
        RCLCPP_INFO(this->get_logger(), "Blue Contours: %ld", blue_contours.size());
        RCLCPP_INFO(this->get_logger(), "Red Contours: %ld", red_contours.size());

        if (!yellow_contours.empty()) {
            RCLCPP_INFO(this->get_logger(), "Detected yellow object");
            twist.angular.z = -0.5;
        } else if (!blue_contours.empty()) {
            RCLCPP_INFO(this->get_logger(), "Detected blue object");
            twist.angular.z = 0.5;
        } else if (red_contours.size() >= 2) {
            RCLCPP_INFO(this->get_logger(), "Detected two red objects");
            twist.linear.x = 0.2;
        } else {
            RCLCPP_INFO(this->get_logger(), "No relevant objects detected");
        }

        cmd_pub_->publish(twist);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
