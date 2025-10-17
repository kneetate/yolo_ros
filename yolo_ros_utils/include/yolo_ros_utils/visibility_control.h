#ifndef YOLO_ROS_UTILS__VISIBILITY_CONTROL_H_
#define YOLO_ROS_UTILS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define YOLO_ROS_UTILS_EXPORT __attribute__ ((dllexport))
    #define YOLO_ROS_UTILS_IMPORT __attribute__ ((dllimport))
  #else
    #define YOLO_ROS_UTILS_EXPORT __declspec(dllexport)
    #define YOLO_ROS_UTILS_IMPORT __declspec(dllimport)
  #endif
  #ifdef YOLO_ROS_UTILS_BUILDING_LIBRARY
    #define YOLO_ROS_UTILS_PUBLIC YOLO_ROS_UTILS_EXPORT
  #else
    #define YOLO_ROS_UTILS_PUBLIC YOLO_ROS_UTILS_IMPORT
  #endif
  #define YOLO_ROS_UTILS_PUBLIC_TYPE YOLO_ROS_UTILS_PUBLIC
  #define YOLO_ROS_UTILS_LOCAL
#else
  #define YOLO_ROS_UTILS_EXPORT __attribute__ ((visibility("default")))
  #define YOLO_ROS_UTILS_IMPORT
  #if __GNUC__ >= 4
    #define YOLO_ROS_UTILS_PUBLIC __attribute__ ((visibility("default")))
    #define YOLO_ROS_UTILS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define YOLO_ROS_UTILS_PUBLIC
    #define YOLO_ROS_UTILS_LOCAL
  #endif
  #define YOLO_ROS_UTILS_PUBLIC_TYPE
#endif

#endif  // YOLO_ROS_UTILS__VISIBILITY_CONTROL_H_
