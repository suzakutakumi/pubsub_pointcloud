#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

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
    this->declare_parameter("voxel_filter_size", -1.0);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "scan", 10, std::bind(&SubscribePointCloudNode::subscriber_callback, this, _1));
  }

private:
  void subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "subscribe pointcloud(size: %d)", msg->data.size());
    RCLCPP_INFO(this->get_logger(), "timestamp %ds %dns", msg->header.stamp.sec, msg->header.stamp.nanosec);
    static pcl::PointCloud<pcl::PointXYZRGB> all_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    pcl::fromROSMsg(*msg, cloud);
    all_cloud.insert(all_cloud.begin(), cloud.begin(), cloud.end());
    all_cloud.is_dense = false;

    auto leaf_size = get_parameter("voxel_filter_size").as_double();
    if (leaf_size > 0.0)
    {
      pcl::VoxelGrid<pcl::PointXYZRGB> sor;
      sor.setInputCloud(all_cloud.makeShared());
      sor.setLeafSize(leaf_size, leaf_size, leaf_size);
      sor.filter(all_cloud);
    }

    auto filepath = "/home/suzaku/ros_choreonoid/pointcloud/";
    auto filename = "merged" + getTimestamp() + ".pcd";

    pcl::io::savePCDFileBinary(filepath + filename, all_cloud);
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