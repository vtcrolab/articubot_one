#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <limits>

class DepthProcessingNode : public rclcpp::Node {
public:
    DepthProcessingNode() : Node("depth_processing"), camera_info_received_(false) {
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_rect_raw", 10,
            std::bind(&DepthProcessingNode::depth_callback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/depth/camera_info", 10,
            std::bind(&DepthProcessingNode::camera_info_callback, this, std::placeholders::_1));

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_model_.fromCameraInfo(*msg);
        camera_info_received_ = true;
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!camera_info_received_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge conversion error: %s", e.what());
            return;
        }

        cv::Mat depth_image = cv_ptr->image;
        int width = depth_image.cols;
        int center_row = depth_image.rows / 2;

        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan->header = msg->header;
        scan->angle_min = -1.57;
        scan->angle_max = 1.57;
        scan->ranges.resize(width, std::numeric_limits<float>::infinity());

        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud->header = msg->header;
        cloud->height = 1;
        cloud->width = width;

        sensor_msgs::PointCloud2Modifier modifier(*cloud);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                         "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                         "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(width);

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

        for (int i = 0; i < width; ++i, ++iter_x, ++iter_y, ++iter_z) {
            float depth = depth_image.at<float>(center_row, i);
            if (std::isfinite(depth)) {
                scan->ranges[i] = depth;
                *iter_x = (i - camera_model_.cx()) * depth / camera_model_.fx();
                *iter_y = 0.0;
                *iter_z = depth;
            }
        }

        scan_pub_->publish(*scan);
        pointcloud_pub_->publish(*cloud);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    image_geometry::PinholeCameraModel camera_model_;
    bool camera_info_received_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthProcessingNode>());
    rclcpp::shutdown();
    return 0;
}
