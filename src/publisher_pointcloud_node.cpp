#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <chrono>
#include <iostream>
#include <fstream>
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
        this->declare_parameter("result_file", "");

        interval = get_parameter("interval").as_int();
        filename = get_parameter("result_file").as_string();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(interval), std::bind(&PublisherPointCloudNode::timer_callback, this));
        if (interval <= 0)
        {
            pose_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("pose_estimation_2D", 10, std::bind(&PublisherPointCloudNode::pose_subscriber_callback, this, _1));
        }
    }

private:
    rclcpp::Clock clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);
    rclcpp::Time before_timestamp;
    std::string filename = "";

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

        sensor_msgs::msg::PointCloud2::UniquePtr msg(new sensor_msgs::msg::PointCloud2());
        pcl::toROSMsg(cloud, *msg);

        RCLCPP_INFO(get_logger(), "Publish pointcloud");
        msg->header.set__frame_id("nemui");
        publisher_->publish(std::move(msg));
        before_timestamp = clock_.now();

        if (interval <= 0)
        {
            timer_->cancel();
            RCLCPP_INFO(get_logger(), "cancel");
        }
    }

    void pose_subscriber_callback(const std_msgs::msg::Float64MultiArray::SharedPtr pose)
    {
        RCLCPP_INFO(this->get_logger(), "subscribe pose");
        rclcpp::Time after_timestamp;
        after_timestamp = clock_.now();

        if (filename != "")
        {
            std::ofstream outputfile(filename, std::ios::app);
            outputfile << pose->data[0] << "," << pose->data[1] << "," << pose->data[2] << "," << pose->data[3] << "," << (after_timestamp - before_timestamp).seconds() << "," << std::endl;
            outputfile.close();
        }

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

        sensor_msgs::msg::PointCloud2::UniquePtr msg(new sensor_msgs::msg::PointCloud2());
        pcl::toROSMsg(cloud, *msg);

        RCLCPP_INFO(get_logger(), "Publish pointcloud");
        msg->header.set__frame_id("nemui");
        publisher_->publish(std::move(msg));
        before_timestamp = clock_.now();
    }
    using PC_Type = pcl::PointCloud<pcl::PointXYZRGB>;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_subscription_;
    int interval;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherPointCloudNode>());
    rclcpp::shutdown();

    return 0;
}