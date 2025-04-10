#ifndef DISTRIBUTE_IMAGE_COMPONENT_HPP
#define DISTRIBUTE_IMAGE_COMPONENT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <memory>// shared ptr使用
#include <functional>// callback使用

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "misora2_distribute_image/cv_mat_type_adapter.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace component_distribute_image
{
class DistributeImage : public rclcpp::Node
{
public:
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;
    std::map<std::string, bool> bool_flags_;
    std::map<std::string, rclcpp::Time> start_times_;
    cv::Mat latest_received_image;

    explicit DistributeImage(const rclcpp::NodeOptions &options);
    DistributeImage() : DistributeImage(rclcpp::NodeOptions{}) {}

private:
    void publish_images();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, std::shared_ptr<rclcpp::Publisher<MyAdaptedType>>> image_publishers_;
    std::map<std::string, std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>>> bool_subscribers_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr received_image_;
    
};
}// namespace component_distribute_image

#endif //DISTRIBUTE_IMAGE_COMPONENT_HPP