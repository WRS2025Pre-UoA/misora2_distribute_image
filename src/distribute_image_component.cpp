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
    received_image_ = this->create_subscription<sensor_msgs::msg::Image>("raw_image",10,std::bind(&DistributeImage::image_callback, this, _1));
    // 減肉用画像を常に受け取り更新する
    received_image_metal_ = this->create_subscription<sensor_msgs::msg::Image>("raw_image_metal",10,std::bind(&DistributeImage::image_metal_callback, this, _1));

    // モードによって用意するsubscriber,publisherのリスト
    std::map<std::string, std::vector<std::string>> topic_list = {
        {"P1",{"pressure","qr"}},
        {"P2",{"pressure","qr"}},
        {"P3",{"cracks","metal_loss"}},
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
            
            // 信号を受けるsubscriber
            bool_subscribers_[list] = this->create_subscription<std_msgs::msg::Bool>(
                list+"_trigger", 1,
                [this, list](const std_msgs::msg::Bool::SharedPtr msg){
                    RCLCPP_INFO_STREAM(this->get_logger(),"Recieved message '"<< msg->data <<"' from: " << list);
                    if(msg->data){
                        bool_flags_[list] = true;
                        start_times_[list] = this->now();
                        // std::unique_ptr<cv::Mat> msg_image = std::make_unique<cv::Mat>(latest_received_image);
                        // RCLCPP_INFO_STREAM(this->get_logger(),"Publish image address: "<< &(msg_image->data));
                        // image_publishers_[list]->publish(std::move(msg_image));
                    }
                    // image_publishers_[list]->publish(latest_received_image);
                }
            );
        }
    }
    else RCLCPP_INFO_STREAM(this->get_logger(),"Not Prepare Publishers and Subscribers: " << param);

    // タイマーで定期的に画像を送信
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_interval_ms), std::bind(&DistributeImage::publish_images, this));
}

void DistributeImage::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    latest_received_image = cv_bridge::toCvCopy(msg, "bgr8")->image;;
    // RCLCPP_INFO_STREAM(this->get_logger(),"Update image");
}

void DistributeImage::image_metal_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    latest_received_image_metal = cv_bridge::toCvCopy(msg, "bgr8")->image;;
    // RCLCPP_INFO_STREAM(this->get_logger(),"Update image");
}

void DistributeImage::publish_images()
{
    for (auto &[key, flag] : bool_flags_)
    {
        if (flag)
        {
            if((this->now() - start_times_[key]).seconds() < check_duration_sec)
            {
                if(key == "metal_loss"){
                    std::unique_ptr<cv::Mat> msg_image_metal = std::make_unique<cv::Mat>(latest_received_image_metal);
                    // RCLCPP_INFO_STREAM(this->get_logger(),"Publish image address: "<< &(msg_image->data));
                    image_publishers_[key]->publish(std::move(msg_image_metal));
                }
                else {
                    std::unique_ptr<cv::Mat> msg_image = std::make_unique<cv::Mat>(latest_received_image);
                    RCLCPP_INFO_STREAM(this->get_logger(),"Publish image address: "<< &(msg_image->data));
                    image_publishers_[key]->publish(std::move(msg_image));
                }
            } else {
                flag = false;
                start_times_[key] = rclcpp::Time(0); // 任意
                // if(key == "metal_loss"){// 送信終了後黒画像を送信する　減肉metal_loss
                cv::Mat black_image = cv::Mat::zeros(480,640,CV_8UC1);
                std::unique_ptr<cv::Mat> msg_image = std::make_unique<cv::Mat>(black_image);
                image_publishers_[key]->publish(std::move(msg_image));
                // }
            }
        }
    }
}
} // namespace component_distribute_image

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_distribute_image::DistributeImage)

