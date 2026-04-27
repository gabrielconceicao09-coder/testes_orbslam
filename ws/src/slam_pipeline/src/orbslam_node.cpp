#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

// ORB-SLAM3
#include "System.h"

class OrbSlamNode : public rclcpp::Node
{
public:
    OrbSlamNode()
    : Node("orbslam_node")
    {
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 ROS2 node started");

        // tópico da câmera
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            10,
            std::bind(&OrbSlamNode::imageCallback, this, std::placeholders::_1)
        );

        // inicializa ORB-SLAM3 (ajuste paths depois)
        slam_ = std::make_shared<ORB_SLAM3::System>(
            "/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt",
            "/opt/ORB_SLAM3/Examples/Monocular/EuRoC.yaml",
            ORB_SLAM3::System::MONOCULAR,
            true
        );
    }

private:

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

            double tframe = msg->header.stamp.sec +
                            msg->header.stamp.nanosec * 1e-9;

            slam_->TrackMonocular(img, tframe);

        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::shared_ptr<ORB_SLAM3::System> slam_;
};

// MAIN OBRIGATÓRIO (erro que você teve antes)
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrbSlamNode>());
    rclcpp::shutdown();
    return 0;
}
