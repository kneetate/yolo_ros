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
        while (rclcpp::ok() && !predict_single_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for PredictSingle service...");
        }
        image_ = cv::imread("src/pruning_ee_camera/yolo_ros/yolo_ros_examples/src/test_image.png");
        image_size_ = image_.size();
        segment(image_);
    }

    void segment(const cv::Mat &image)
    {
        if (!predict_single_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "PredictSingle service not available.");
            return;
        }

        auto single_request = std::make_shared<yolo_msgs::srv::PredictSingle::Request>();
        single_request->image = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg());
        RCLCPP_INFO(this->get_logger(), "Sending PredictSingle request.");
        auto single_future = predict_single_client_->async_send_request(single_request,
                                                                        std::bind(&YoloRosExampleNode::single_prediction_callback, this, std::placeholders::_1));
    }

    void single_prediction_callback(rclcpp::Client<yolo_msgs::srv::PredictSingle>::SharedFuture future)
    {
        if (future.valid())
        {
            RCLCPP_INFO(this->get_logger(), "Received PredictSingle response.");
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "Clustering detections...");
            RCLCPP_INFO(this->get_logger(), "Number of detections before clustering: %zu", result->detections.detections.size());
            cv::Mat output_image_before = yolo_ros_utils::YoloRosUtils::draw_detections_on_image(
                image_, result->detections);
            cv::imwrite("detections_before_clustering.png", output_image_before);
            // auto clustered = clustering(result);
            // RCLCPP_INFO(this->get_logger(), "Number of detections after clustering: %zu", clustered.detections.size());
            // // save each mask as image
            // for (size_t i = 0; i < clustered.detections.size(); ++i)
            // {
            //     cv::Mat mask = cv::Mat::zeros(image_size_, CV_8UC1);
            //     for (const auto &pt : clustered.detections[i].mask.data)
            //     {
            //         mask.at<uchar>(pt.y, pt.x) = 255;
            //     }
            //     cv::imwrite("mask_" + std::to_string(i) + ".png", mask);
            // }
            // cv::Mat output_image = yolo_ros_utils::YoloRosUtils::draw_detections_on_image(
            //     image_, clustered);
            // cv::imwrite("clustered_detections.png", output_image);
            // RCLCPP_INFO(this->get_logger(), "Single prediction successful.");
        }
    }

private:
    rclcpp::Client<yolo_msgs::srv::PredictSingle>::SharedPtr predict_single_client_;
    cv::Size image_size_{640, 480};
    cv::Mat image_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YoloRosExampleNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}