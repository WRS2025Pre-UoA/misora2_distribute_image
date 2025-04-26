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
    std::map<std::string, bool> bool_flags_;// 連億処理信号とそれに対応する送り先のmap
    std::map<std::string, rclcpp::Time> start_times_;// 信号を受け取ってから1secとカウントするため
    cv::Mat latest_received_image;// 最新の画像を保存
    double check_duration_sec;

    explicit DistributeImage(const rclcpp::NodeOptions &options);
    DistributeImage() : DistributeImage(rclcpp::NodeOptions{}) {}

private:
    void publish_images();// 信号に対応するところへ画像を流す
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);// 定期的にmisoraから送られてくる画像の保存を行う処理関数

    rclcpp::TimerBase::SharedPtr timer_;// 信号が来たら1sec間画像を流す
    std::map<std::string, std::shared_ptr<rclcpp::Publisher<MyAdaptedType>>> image_publishers_;// 画像を流す
    std::map<std::string, std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>>> bool_subscribers_;// 連続処理信号を受け取る
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr received_image_;// misoraからの画像を受け取る
    
};
}// namespace component_distribute_image

#endif //DISTRIBUTE_IMAGE_COMPONENT_HPP