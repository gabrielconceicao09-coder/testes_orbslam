#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "System.h"

class OrbSlamNode : public rclcpp::Node
{
public:
    OrbSlamNode() : Node("orbslam_node")
    {
        RCLCPP_INFO(get_logger(), "ORB-SLAM3 node started");

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&OrbSlamNode::imageCallback, this, std::placeholders::_1)
        );

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/orbslam/pose", 10
        );

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
        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

        double t = msg->header.stamp.sec +
                    msg->header.stamp.nanosec * 1e-9;

        // ORB-SLAM3 tracking
        Sophus::SE3f Tcw = slam_->TrackMonocular(img, t);

        publishPose(Tcw, msg->header.stamp);
    }

    void publishPose(const Sophus::SE3f &Tcw, builtin_interfaces::msg::Time stamp)
    {
        auto Twc = Tcw.inverse();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = "map";

        pose.pose.position.x = Twc.translation().x();
        pose.pose.position.y = Twc.translation().y();
        pose.pose.position.z = Twc.translation().z();

        auto q = Twc.unit_quaternion();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pose_pub_->publish(pose);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<ORB_SLAM3::System> slam_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrbSlamNode>());
    rclcpp::shutdown();
    return 0;
}
