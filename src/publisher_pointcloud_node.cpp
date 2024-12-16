#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <chrono>
#include <iostream>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

class PublisherPointCloudNode : public rclcpp::Node
{
public:
    PublisherPointCloudNode()
        : Node("publisher_pointcloud_node")
    {
        this->declare_parameter("interval", 1);
        this->declare_parameter("filepath", "");
        this->declare_parameter("voxel_filter_size", std::vector<double>{});

        interval = get_parameter("interval").as_int();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(interval), std::bind(&PublisherPointCloudNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        std::string file = get_parameter("filepath").as_string();
        if (file == "")
        {
            return;
        }

        PC_Type cloud;
        pcl::io::loadPCDFile(file, cloud);

        auto leaf_size = get_parameter("voxel_filter_size").as_double_array();
        if (leaf_size.size() >= 3)
        {
            pcl::VoxelGrid<PC_Type::PointType> sor;
            sor.setInputCloud(cloud.makeShared());
            sor.setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
            sor.filter(cloud);
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);

        RCLCPP_INFO(get_logger(), "Publish pointcloud");
        msg.header.set__frame_id("nemui");
        publisher_->publish(msg);

        if (interval <= 0)
        {
            timer_->cancel();
            RCLCPP_INFO(get_logger(), "cancel");
        }
    }

    using PC_Type = pcl::PointCloud<pcl::PointXYZRGB>;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int interval;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherPointCloudNode>());
    rclcpp::shutdown();

    return 0;
}