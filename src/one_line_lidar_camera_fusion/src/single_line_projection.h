// copyright 2021.02.04 <YWY>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <time.h>
#include <boost/thread/thread.hpp>
#include "my_structure.h"

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// boundingboxes with dist
// #include "one_line_lidar_camera_fusion/BoundingBoxesWithDist.h"
// #include "one_line_lidar_camera_fusion/BoundingBoxWithDist.h"

//jsk_recognition boundingbox
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

//perception_msgs/LidarObstacle
#include <perception_msgs/LidarObstacle.h>
#include <perception_msgs/LidarObstacles.h>




class SingleLineProjection {
 private:

  float three_d_box_x_;
  float three_d_box_y_;
  PixelCoor CalculatePixelCoor(Eigen::Vector3f cloud_point);

  one_line_lidar_camera_fusion::BoundingBoxWithDist CalculateCubeDist
    (const darknet_ros_msgs::BoundingBox &box, MyPoint* points, cv::Mat& img);

  float CalculateThreeDBoxX
    (const darknet_ros_msgs::BoundingBox& box, MyPoint* points);

  float CalculateThreeDBoxY
    (const darknet_ros_msgs::BoundingBox& box, MyPoint* points);

  void DrawCube(cv::Mat& img, cv::Rect r);

  void DrawCube(cv::Mat& img, cv::Rect r, std::string dist, std::string object);

  void DrawCircle(cv::Mat& raw_img, cv::Point const& circle_center, double const& point_dist);

 public:
  Eigen::Matrix3f world_to_cam_r_;
  Eigen::Vector3f world_to_cam_t_;

  void LoadRotationMatrix(const std::string& config_path);

  void LoadTransformVector(const std::string& config_path);

  one_line_lidar_camera_fusion::BoundingBoxesWithDist dist_boxes_;
  jsk_recognition_msgs::BoundingBox jsk_box_;
  jsk_recognition_msgs::BoundingBoxArray jsk_boxes_;
  perception_msgs::LidarObstacle lidar_obstacle_;
  perception_msgs::LidarObstacles lidar_obstacles_;

  SingleLineProjection();

  void LidarProjection2Camera(cv::Mat& raw_img, int box_num,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud,
                        const darknet_ros_msgs::BoundingBoxes &bbxes);
};
