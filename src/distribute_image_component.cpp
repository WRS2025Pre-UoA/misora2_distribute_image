#include "misora2_distribute_image/distribute_image_component.hpp"

namespace component_distribute_image
{
DistributeImage::DistributeImage(const rclcpp::NodeOptions &options)
    : Node("distribute_image",options)
{
    // P1~4,P6それぞれのタスクに対応
    this->declare_parameter("mode","P6");
    std::string param = this->get_parameter("mode").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(),"Received mode parameter: " << param);

    // 画像送信時間、間隔をパラメータにする
    this->declare_parameter<double>("check_duration_sec",1.0);
    this->declare_parameter<int>("timer_interval_ms",100);

    int timer_interval_ms;
    this->get_parameter("check_duration_sec", check_duration_sec);
    this->get_parameter("timer_interval_ms", timer_interval_ms);
    RCLCPP_INFO_STREAM(this->get_logger(),"Interval: " << timer_interval_ms << " ms, duration_time: " << check_duration_sec);

    // 常に画像を受け取り更新する
    received_image_ = this->create_subscription<MyAdaptedType>("raw_image",10,std::bind(&DistributeImage::image_callback, this, _1));
    // 減肉用画像を常に受け取り更新する
    // received_image_metal_ = this->create_subscription<sensor_msgs::msg::Image>("raw_image_metal",10,std::bind(&DistributeImage::image_metal_callback, this, _1));

    // モードによって用意するsubscriber,publisherのリスト
    std::map<std::string, std::vector<std::string>> topic_list = {
        {"P1",{"pressure","qr"}},
        {"P2",{"pressure","qr"}},
        {"P3",{"qr","cracks"}},
        {"P4",{"qr"}},
        {"P6",{"pressure","qr"}}
    };
    // publisher,subscriber作成
    if(topic_list.find(param) != topic_list.end()){
        for(const auto &list : topic_list[param]){
            // 対応するノードに画像を送信するpublisherリスト
            image_publishers_[list] = this->create_publisher<MyAdaptedType>(list+"_image",10);

            bool_flags_[list] = false; // 初期状態は false
            start_times_[list] = rclcpp::Time(0);
        }
    }
    else RCLCPP_INFO_STREAM(this->get_logger(),"Not Prepare Publishers: " << param);

    // 信号を受けるsubscriber
    trigger_subscribers_ = this->create_subscription<std_msgs::msg::String>(
        "triggers", 1,
        [this](const std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO_STREAM(this->get_logger(),"Recieved message '"<< msg->data <<"' from: Operator" );
            std::string key = msg->data;
            if(!key.empty() && bool_flags_.find(key) != bool_flags_.end()){
                bool_flags_[key] = true;
                start_times_[key] = this->now();
            }
        });
    

    // タイマーで定期的に画像を送信
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_interval_ms), std::bind(&DistributeImage::publish_images, this));
}

// void DistributeImage::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
//     // msg の encoding を確認
//     if (msg->encoding == "rgb8") {
//         latest_received_image = cv_bridge::toCvCopy(msg, "rgb8")->image;
//         cv::cvtColor(latest_received_image, latest_received_image, cv::COLOR_RGB2BGR);
//         RCLCPP_DEBUG(this->get_logger(), "Converted RGB8 -> BGR");
//     } 
//     else if (msg->encoding == "bgr8") {
//         latest_received_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
//         RCLCPP_DEBUG(this->get_logger(), "Received BGR8 -> No conversion");
//     } 
//     else {
//         // 想定外のエンコードの場合はとりあえず BGR に変換
//         latest_received_image = cv_bridge::toCvCopy(msg, msg->encoding)->image;
//         cv::cvtColor(latest_received_image, latest_received_image, cv::COLOR_RGB2BGR);
//         RCLCPP_WARN(this->get_logger(), "Unexpected encoding %s -> Forced to BGR", msg->encoding.c_str());
//     }
// }

void DistributeImage::image_callback(const cv::Mat & img) {
    // ここに来る時点で TypeAdapter により bgr8 に変換済み
    latest_received_image = img.clone();  

    RCLCPP_DEBUG(this->get_logger(), "Received image (BGR, %d x %d)", 
                 img.cols, img.rows);
}


void DistributeImage::publish_images()
{
    for (auto &[key, flag] : bool_flags_)
    {
        if (flag)
        {
            if((this->now() - start_times_[key]).seconds() < check_duration_sec)
            {
                std::unique_ptr<cv::Mat> msg_image = std::make_unique<cv::Mat>(latest_received_image);
                RCLCPP_INFO_STREAM(this->get_logger(),"Publish image to: "<< key);
                image_publishers_[key]->publish(std::move(msg_image));
                
            } else {
                flag = false;
                start_times_[key] = rclcpp::Time(0); // 任意
                cv::Mat black_image = cv::Mat::zeros(480,640,CV_8UC1);
                std::unique_ptr<cv::Mat> msg_image = std::make_unique<cv::Mat>(black_image);
                image_publishers_[key]->publish(std::move(msg_image));
                
            }
        }
    }
}
} // namespace component_distribute_image

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_distribute_image::DistributeImage)

