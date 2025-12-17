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
        const yolo_msgs::msg::DetectionArray &detection_array, bool different_color)
    {
        cv::Mat output_image = image.clone();
        std::vector<cv::Scalar> colors = YoloRosUtils::create_random_colors(detection_array.detections.size());
        int color_index = 0;
        for (const auto &detection : detection_array.detections)
        {
            cv::Scalar color = different_color ? colors[color_index++] : cv::Scalar(0, 255, 0);
            cv::Rect box(cv::Point(detection.bbox.center.position.x - detection.bbox.size.x / 2, detection.bbox.center.position.y - detection.bbox.size.y / 2),
                         cv::Size(detection.bbox.size.x, detection.bbox.size.y));
            cv::rectangle(output_image, box, color, 2);
            cv::Mat mask_mat = cv::Mat::zeros(output_image.rows, output_image.cols, CV_8UC3);
            //  detection.mask.dataに内包される領域点を塗りつぶす
            std::vector<cv::Point> points_mat(detection.mask.data.size());
            for (int i = 0; i < detection.mask.data.size(); ++i)
            {
                if (i > 0)
                {
                    cv::line(mask_mat,
                             cv::Point(detection.mask.data[i - 1].x, detection.mask.data[i - 1].y),
                             cv::Point(detection.mask.data[i].x, detection.mask.data[i].y),
                             color, 2);
                }
                points_mat[i] = cv::Point(detection.mask.data[i].x, detection.mask.data[i].y);
            }
            cv::fillConvexPoly(mask_mat, points_mat, color, cv::LINE_AA);
            std::string label = detection.class_name + ": " + std::to_string(detection.score);
            int baseLine;
            cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::rectangle(output_image, cv::Point(box.x, box.y - label_size.height),
                          cv::Point(box.x + label_size.width, box.y + baseLine),
                          color, cv::FILLED);
            cv::putText(output_image, label, cv::Point(box.x, box.y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            cv::addWeighted(output_image, 1.0, mask_mat, 0.8, 0, output_image);
        }
        return output_image;
    }

    cv::Mat YoloRosUtils::draw_detections_on_image(
        const sensor_msgs::msg::Image &image_msg,
        const yolo_msgs::msg::DetectionArray &detection_array, bool different_color)
    {
        cv::Mat cv_image = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(image_msg), "bgr8")->image;
        return draw_detections_on_image(cv_image, detection_array, different_color);
    }

    std::vector<cv::Scalar> YoloRosUtils::create_random_colors(const int &num_colors)
    {
        std::vector<cv::Scalar> colors;
        cv::RNG rng(12345);
        for (int i = 0; i < num_colors; ++i)
        {
            int r = rng.uniform(0, 256);
            int g = rng.uniform(0, 256);
            int b = rng.uniform(0, 256);
            colors.push_back(cv::Scalar(b, g, r));
        }
        return colors;
    }
} // namespace yolo_ros_utils
