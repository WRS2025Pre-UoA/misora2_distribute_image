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

    // 常に画像を受け取り更新する
    received_image_ = this->create_subscription<sensor_msgs::msg::Image>("raw_image",10,std::bind(&DistributeImage::image_callback, this, _1));

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
                list+"_trigger", 10,
                [this, list](const std_msgs::msg::Bool::SharedPtr msg){
                    RCLCPP_INFO_STREAM(this->get_logger(),"Recieved message '"<< msg->data <<"' from: " << list);
                    if(msg->data){
                        bool_flags_[list] = true;
                        start_times_[list] = this->now();
                    }
                    // image_publishers_[list]->publish(latest_received_image);
                }
            );
        }
    }
    else RCLCPP_INFO_STREAM(this->get_logger(),"Not Prepare Publishers and Subscribers: " << param);

    // タイマーで定期的に画像を送信
    timer_ = this->create_wall_timer(100ms, std::bind(&DistributeImage::publish_images, this));
}
void DistributeImage::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    latest_received_image = cv_bridge::toCvCopy(msg, "bgr8")->image;;
    // RCLCPP_INFO_STREAM(this->get_logger(),"Update image");
}
void DistributeImage::publish_images()
{
    for (auto &[key, flag] : bool_flags_)
    {
        if (flag)
        {
            if((this->now() - start_times_[key]).seconds() < 1.0)
            {
                std::unique_ptr<cv::Mat> msg_image = std::make_unique<cv::Mat>(latest_received_image);
                RCLCPP_INFO_STREAM(this->get_logger(),"Publish image address: "<< &(msg_image->data));
                image_publishers_[key]->publish(std::move(msg_image));
            } else {
                flag = false;
                start_times_[key] = rclcpp::Time(0); // 任意
            }
        }
    }
}
} // namespace component_distribute_image

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_distribute_image::DistributeImage)

