#ifndef YOLO_ROS_UTILS__YOLO_ROS_UTILS_HPP_
#define YOLO_ROS_UTILS__YOLO_ROS_UTILS_HPP_

#include "yolo_ros_utils/visibility_control.h"
#include <yolo_msgs/msg/detection.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

namespace yolo_ros_utils
{

  class YoloRosUtils
  {
  public:
    YoloRosUtils();
    static cv::Mat draw_detections_on_image(
        const cv::Mat &image,
        const yolo_msgs::msg::DetectionArray &detection_array, bool different_color = false);
    static cv::Mat draw_detections_on_image(
        const sensor_msgs::msg::Image &image_msg,
        const yolo_msgs::msg::DetectionArray &detection_array, bool different_color = false);
    static std::vector<cv::Scalar> create_random_colors(const int &num_colors);
    virtual ~YoloRosUtils();
  };

} // namespace yolo_ros_utils

#endif // YOLO_ROS_UTILS__YOLO_ROS_UTILS_HPP_
