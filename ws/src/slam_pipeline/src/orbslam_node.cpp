#include <memory>
#include <queue>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "System.h"

class OrbSlamNode : public rclcpp::Node
{
public:
    OrbSlamNode()
    : Node("orbslam_node")
    {
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 ROS2 node started");

        // SUBSCRIBER (só enfileira imagens)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            10,
            std::bind(&OrbSlamNode::imageCallback, this, std::placeholders::_1)
        );

        // PUBLISHER POSE
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/orbslam/pose", 10
        );

        // SLAM INIT
        slam_ = std::make_shared<ORB_SLAM3::System>(
            "/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt",
            "/opt/ORB_SLAM3/Examples/Monocular/EuRoC.yaml",
            ORB_SLAM3::System::MONOCULAR,
            true
        );

        // TIMER de processamento (evita travar callback ROS)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&OrbSlamNode::process, this)
        );
    }

private:

    // =========================
    // CALLBACK: só enfileira
    // =========================
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        img_buffer_.push(msg);
    }

    // =========================
    // PROCESSAMENTO SLAM
    // =========================
    void process()
    {
        sensor_msgs::msg::Image::SharedPtr msg;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (img_buffer_.empty())
                return;

            msg = img_buffer_.front();
            img_buffer_.pop();
        }

        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

        double t =
            msg->header.stamp.sec +
            msg->header.stamp.nanosec * 1e-9;

        auto Tcw = slam_->TrackMonocular(img, t);

        // DEBUG IMPORTANTE
        auto state = slam_->GetTrackingState();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Tracking state: %d", state
        );

        if (Tcw.empty() || state != 2)
        {
            return; // ainda não está OK
        }

        publishPose(Tcw, msg->header.stamp);
    }

    // =========================
    // PUBLICAÇÃO DE POSE
    // =========================
    void publishPose(const Sophus::SE3f &Tcw,
                     builtin_interfaces::msg::Time stamp)
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

private:
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // SLAM
    std::shared_ptr<ORB_SLAM3::System> slam_;

    // BUFFER THREAD-SAFE
    std::queue<sensor_msgs::msg::Image::SharedPtr> img_buffer_;
    std::mutex mutex_;
};

// =========================
// MAIN
// =========================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrbSlamNode>());
    rclcpp::shutdown();
    return 0;
}
