#include "yolo_ros_utils/yolo_ros_utils.hpp"

namespace yolo_ros_utils
{

    YoloRosUtils::YoloRosUtils()
    {
    }

    YoloRosUtils::~YoloRosUtils()
    {
    }

    cv::Mat YoloRosUtils::draw_detections_on_image(
        const cv::Mat &image,
        const yolo_msgs::msg::DetectionArray &detection_array)
    {
        cv::Mat output_image = image.clone();
        for (const auto &detection : detection_array.detections)
        {
            cv::Rect box(cv::Point(detection.bbox.center.position.x, detection.bbox.center.position.y),
                         cv::Size(detection.bbox.size.x, detection.bbox.size.y));
            cv::rectangle(output_image, box, cv::Scalar(0, 255, 0), 2);
            cv::Mat mask_mat = cv::Mat::zeros(output_image.rows, output_image.cols, CV_8UC3);
            for (const auto &mask_pt : detection.mask.data)
            {
                mask_mat.at<cv::Vec3b>(static_cast<int>(mask_pt.y), static_cast<int>(mask_pt.x)) = cv::Vec3b(0, 255, 0);
            }
            std::string label = detection.class_name + ": " + std::to_string(detection.score);
            int baseLine;
            cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::rectangle(output_image, cv::Point(box.x, box.y - label_size.height),
                          cv::Point(box.x + label_size.width, box.y + baseLine),
                          cv::Scalar(0, 255, 0), cv::FILLED);
            cv::putText(output_image, label, cv::Point(box.x, box.y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            cv::addWeighted(output_image, 1.0, mask_mat, 0.3, 0, output_image);
        }
        return output_image;
    }

    cv::Mat YoloRosUtils::draw_detections_on_image(
        const sensor_msgs::msg::Image &image_msg,
        const yolo_msgs::msg::DetectionArray &detection_array)
    {
        cv::Mat cv_image = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(image_msg), "bgr8")->image;
        return draw_detections_on_image(cv_image, detection_array);
    }
} // namespace yolo_ros_utils
