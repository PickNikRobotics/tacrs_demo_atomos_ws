#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

class DepthPointPublisher : public rclcpp::Node {
public:
    DepthPointPublisher() : Node("depth_point_publisher") {
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/scene_camera/depth/camera_info", rclcpp::SensorDataQoS(),
            std::bind(&DepthPointPublisher::cameraInfoCallback, this, std::placeholders::_1));

        depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/scene_camera/depth/image_rect_raw", rclcpp::SensorDataQoS(),
            std::bind(&DepthPointPublisher::depthImageCallback, this, std::placeholders::_1));

        point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/depth_center_point", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;

    image_geometry::PinholeCameraModel cam_model_;
    bool camera_info_received_ = false;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        cam_model_.fromCameraInfo(msg);
        camera_info_received_ = true;
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!camera_info_received_) return;

        // Convert image
        cv::Mat depth;
        try {
            depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        int cx = depth.cols / 2;
        int cy = depth.rows / 2;

        float depth_value = depth.at<uint16_t>(cy, cx) * 0.001f; // Convert mm to meters
        if (depth_value <= 0.0 || std::isnan(depth_value)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Invalid depth at center.");
            return;
        }

        cv::Point3d ray = cam_model_.projectPixelTo3dRay(cv::Point2d(cx, cy));
        cv::Point3d point = ray * depth_value;

        geometry_msgs::msg::PointStamped msg_out;
        msg_out.header = msg->header;
        msg_out.point.x = point.x;
        msg_out.point.y = point.y;
        msg_out.point.z = point.z;

        point_pub_->publish(msg_out);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthPointPublisher>());
    rclcpp::shutdown();
    return 0;
}