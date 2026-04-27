#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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

        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/orbslam/map_points", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&OrbSlamNode::publishMap, this));
                
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

    void publishMap()
    {
        auto mapPoints = mpSLAM->GetTrackedMapPoints();  
        // dependendo do wrapper pode ser GetAllMapPoints()
    
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = this->now();
    
        cloud_msg.height = 1;
        cloud_msg.width = mapPoints.size();
        cloud_msg.is_dense = false;
    
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(
            3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32
        );
    
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    
        for (auto& p : mapPoints)
        {
            if (!p) continue;
    
            cv::Mat pos = p->GetWorldPos();
    
            *iter_x = pos.at<float>(0);
            *iter_y = pos.at<float>(1);
            *iter_z = pos.at<float>(2);
    
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
    
        map_pub_->publish(cloud_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
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
