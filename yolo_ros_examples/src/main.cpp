#include <yolo_msgs/srv/predict_multiple.hpp>
#include <yolo_msgs/srv/predict_single.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_ros_utils/yolo_ros_utils.hpp>

class YoloRosExampleNode : public rclcpp::Node
{
public:
    YoloRosExampleNode(const rclcpp::NodeOptions &options)
        : Node("yolo_ros_example_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "YoloRosExampleNode initialized.");
        predict_single_client_ = this->create_client<yolo_msgs::srv::PredictSingle>("yolo/predict_single");
        predict_multiple_client_ = this->create_client<yolo_msgs::srv::PredictMultiple>("yolo/predict_multiple");
        trigger_service_single_ = this->create_service<std_srvs::srv::Trigger>(
            "yolo_trigger_service_single",
            std::bind(&YoloRosExampleNode::trigger_callback_single, this, std::placeholders::_1, std::placeholders::_2));
        trigger_service_multiple_ = this->create_service<std_srvs::srv::Trigger>(
            "yolo_trigger_service_multiple",
            std::bind(&YoloRosExampleNode::trigger_callback_multiple, this, std::placeholders::_1, std::placeholders::_2));
    }
    void trigger_callback_single(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;                                                                               // 未使用の引数を抑制
        cv::Mat image = cv::imread("src/pruning_ee_camera/yolo_ros/yolo_ros_examples/src/test_image.png"); // ダミーの画像読み込み

        if (!predict_single_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "PredictSingle service not available.");
            response->success = false;
            response->message = "PredictSingle service not available.";
            return;
        }

        auto single_request = std::make_shared<yolo_msgs::srv::PredictSingle::Request>();
        single_request->image = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg());
        RCLCPP_INFO(this->get_logger(), "Sending PredictSingle request.");
        auto single_future = predict_single_client_->async_send_request(single_request,
                                                                        std::bind(&YoloRosExampleNode::single_prediction_callback, this, std::placeholders::_1));
    }

    void trigger_callback_multiple(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;                                                                               // 未使用の引数を抑制
        cv::Mat image = cv::imread("src/pruning_ee_camera/yolo_ros/yolo_ros_examples/src/test_image.png"); // ダミーの画像読み込み
        auto multiple_request = std::make_shared<yolo_msgs::srv::PredictMultiple::Request>();
        for (int i = 0; i < 10; ++i)
        {
            multiple_request->images.push_back(*(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg()));
        }
        RCLCPP_INFO(this->get_logger(), "Sending PredictMultiple request.");
        auto multiple_future = predict_multiple_client_->async_send_request(multiple_request,
                                                                            std::bind(&YoloRosExampleNode::multiple_prediction_callback, this, std::placeholders::_1));
    }

    void single_prediction_callback(rclcpp::Client<yolo_msgs::srv::PredictSingle>::SharedFuture future)
    {
        if (future.valid())
        {
            RCLCPP_INFO(this->get_logger(), "Received PredictSingle response.");
            auto result = future.get();
            cv::Mat result_image = yolo_ros_utils::YoloRosUtils::draw_detections_on_image(
                cv::imread("src/pruning_ee_camera/yolo_ros/yolo_ros_examples/src/test_image.png"), // ダミーの画像読み込み
                result->detections);
            cv::imshow("Yolo Single Prediction", result_image);
            cv::waitKey(0);
            RCLCPP_INFO(this->get_logger(), "Single prediction successful.");
        }
    }

    void multiple_prediction_callback(rclcpp::Client<yolo_msgs::srv::PredictMultiple>::SharedFuture future)
    {
        if (future.valid())
        {
            RCLCPP_INFO(this->get_logger(), "Received PredictMultiple response.");
            auto result = future.get();
            for (size_t i = 0; i < result->detections.size(); ++i)
            {
                cv::Mat result_image = yolo_ros_utils::YoloRosUtils::draw_detections_on_image(
                    cv::imread("src/pruning_ee_camera/yolo_ros/yolo_ros_examples/src/test_image.png"), // ダミーの画像読み込み
                    result->detections[i]);
                cv::imshow("Yolo Multiple Prediction " + std::to_string(i), result_image);
            }
            cv::waitKey(0);
            RCLCPP_INFO(this->get_logger(), "Multiple prediction successful.");
        }
    }

private:
    rclcpp::Client<yolo_msgs::srv::PredictSingle>::SharedPtr predict_single_client_;
    rclcpp::Client<yolo_msgs::srv::PredictMultiple>::SharedPtr predict_multiple_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_single_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_multiple_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YoloRosExampleNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}