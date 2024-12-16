#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

#include <chrono>
#include <iostream>
#include <sstream>

std::string getTimestamp()
{
  // 現在のUnixタイム（エポック秒）を取得
  const auto &now = std::chrono::system_clock::now();
  const auto &currentTime = std::chrono::system_clock::to_time_t(now);

  // Unixタイムを文字列に変換
  std::stringstream ss;
  ss << currentTime;

  return ss.str();
}

using std::placeholders::_1;

class SubscribePointCloudNode : public rclcpp::Node
{
public:
  SubscribePointCloudNode()
      : Node("subscribe_pointcloud_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "scan", 10, std::bind(&SubscribePointCloudNode::subscriber_callback, this, _1));
  }

private:
  void subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "subscribe pointcloud(size: %d)", msg->data.size());
    RCLCPP_INFO(this->get_logger(), "timestamp %ds %dns", msg->header.stamp.sec, msg->header.stamp.nanosec);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);
    auto filepath = "/home/suzaku/ros_choreonoid/pointcloud/";
    auto filename = "merged" + getTimestamp() + ".pcd";

    pcl::io::savePCDFileBinary(filepath + filename, cloud);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscribePointCloudNode>());
  rclcpp::shutdown();

  return 0;
}