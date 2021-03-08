// Copyright 2021.02.04 <YWY>
#include <ros/ros.h>
#include "single_line_projection.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <jsk_recognition_msgs/BoundingBox.h>


cv::Mat g_receive_image;
pcl::PointCloud<pcl::PointXYZ> g_cloud;
image_transport::Publisher g_image_pub;
bool g_image_empty = true;
bool g_lidar_empty = true;
ros::Publisher g_dist_boxes_pub;
ros::Publisher g_jsk_boxes_pub;
ros::Publisher g_jsk_box_pub;
ros::Publisher g_lidar_obstacles_pub;

SingleLineProjection g_projector;

//###############################################################
// 排序函数，在NMS中使用，对boundingboxes通过置信度进行排序
void MySort(int n, const darknet_ros_msgs::BoundingBoxes& bdxes,
std::vector<int> indices) {
  for (int i = 0; i < n; i++) {
    for (int j = i+1; j < n; j++) {
      if (bdxes.bounding_boxes[indices[j]].probability >
          bdxes.bounding_boxes[indices[i]].probability) {
        int index_tmp = indices[i];
        indices[i] = indices[j];
        indices[j] = index_tmp;
      }
    }
  }
}

// Non Maximum Supression 非极大值抑制，对检测出的boundingboxes重叠部分进行筛选
// 重叠面积超过thresh hold的检测框中，选择probability最大的进行保留，其他删除
darknet_ros_msgs::BoundingBoxes
NMS(const darknet_ros_msgs::BoundingBoxes& bdxes) {
  int box_num = bdxes.bounding_boxes.size();
  std::vector<float> box_area(box_num);  // 定义窗口面积变量并分配空间
  std::vector<int> indices(box_num);  // 定义窗口索引并分配空间
  std::vector<int> is_supressed(box_num);  // 定义是否抑制flag

  if (box_num <= 1) {
    return bdxes;
  }

  // 初始化indices, box_area, is_supressed信息
  for (int i = 0; i < box_num; i++) {
    indices[i] = i;
    is_supressed[i] = 0;
    // 计算面积 s=(xmax-xmin)*(ymax-ymin)
    float width = bdxes.bounding_boxes[i].xmax-bdxes.bounding_boxes[i].xmin;
    float height = bdxes.bounding_boxes[i].ymax-bdxes.bounding_boxes[i].ymin;
    box_area[i] = width*height;
  }

  // 对输出窗口按照probability进行排序
  MySort(box_num, bdxes, indices);


  // 处理排序之后的数组 indicies[0]是probability最大的一个bbx，所有从它开始
  // 并且所有bbx都会被遍历，至少probability最大的bbx不会被 suppress
  for (int i = 0; i < box_num-1; i++) {
    if (!is_supressed[indices[i]]) {
      int j = i+1;
      while (j < box_num) {
        if (!is_supressed[indices[j]]) {  // 如果未被抑制，求重叠部分
          // 左边 x 最大值
          int x_left_max = std::max(bdxes.bounding_boxes[indices[i]].xmin,
                                    bdxes.bounding_boxes[indices[j]].xmin);
          // 右边 x 最小值
          int x_right_min = std::min(bdxes.bounding_boxes[indices[i]].xmax,
                                     bdxes.bounding_boxes[indices[j]].xmax);
          // 下边 y 最大值
          int y_buttom_max = std::max(bdxes.bounding_boxes[indices[i]].ymin,
                                      bdxes.bounding_boxes[indices[j]].ymin);
          // 上边 y 最小值
          int y_top_min = std::min(bdxes.bounding_boxes[indices[i]].ymax,
                                   bdxes.bounding_boxes[indices[j]].ymax);

          int overlap_width = x_right_min-x_left_max;
          int overlap_height = y_top_min-y_buttom_max;
          if (overlap_height > 0 && overlap_width > 0) {
            float overlap_part =
                    (overlap_width*overlap_height)/box_area[indices[j]];
            if (overlap_part > 0.7) {    // 如果重叠区域超过阈值0.7
              is_supressed[indices[j]] = 1;  // 将bbx[j]标记为抑制
            }
          }
        }
        j++;
      }
    }
  }

  int box_out_num = 0;
  for (int i = 0; i < box_num; i++) {
    if (!is_supressed[i]) {
      box_out_num++;  // 统计输出的窗口数
    }
  }
  int index = 0;

  darknet_ros_msgs::BoundingBoxes bdxes_nms;
  for (int i = 0; i < box_num; i++) {  // 遍历所有bbx
    // 如果没有被抑制，就push进入 bdxes_nms 中
    if (!is_supressed[i] && bdxes.bounding_boxes[i].probability > 0.5) {
      bdxes_nms.bounding_boxes.push_back(bdxes.bounding_boxes[i]);
    }
  }
  return bdxes_nms;
}

void VideoCallback(const sensor_msgs::ImageConstPtr& img_msg) {
  g_image_empty = false;
  cv::Mat image;
  cv::Mat undistort_image;
  // 3x3 matrix-> intrinsic_eigen
  cv::Mat intrinsic_eigen = (cv::Mat_<double>(3, 3)
            << 896.4223, 0, 649.2941, 0, 895.2755, 328.0513, 0, 0, 1);

  // distortion_eigen  (k1, k2, p1, p2, K3)
  cv::Mat distort_eigen = (cv::Mat_<double>(1, 5)
            << -0.4433, 0.2447, -0.0009894, -0.0000488, -0.0836);

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
  }
  catch (cv_bridge::Exception e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
               img_msg->encoding.c_str());
    return;
  }
  image = cv_ptr -> image;
  if (!image.empty()) {
    cv::undistort(image, undistort_image, intrinsic_eigen, distort_eigen);
    // image dedistortion
    image = undistort_image;

    cv::resize(image, g_receive_image, cv::Size(1280, 720));
  }
}

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  g_lidar_empty = false;
  sensor_msgs::PointCloud2 cloud;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;

  projector_.transformLaserScanToPointCloud("laser", *scan, cloud, tfListener_);

  pcl::fromROSMsg(cloud, g_cloud);
}

void BbxCallback(const darknet_ros_msgs::BoundingBoxes& bdxes) {
  if (bdxes.bounding_boxes.size() <= 0) {
    return;
  }

  if (g_lidar_empty) {
    return;
  }

  if (g_image_empty) {
    return;
  }

  darknet_ros_msgs::BoundingBoxes bdxes_nms = NMS(bdxes);

  // one_line_lidar_camera_fusion::BoundingBoxesWithDist dist_boxes;
  ThreeDBoundingBox three_d_box;
  g_projector.LidarProjection2Camera(g_receive_image,
                                     bdxes_nms.bounding_boxes.size(),
                                     g_cloud.makeShared(), bdxes_nms);
                                     // 我只需要把这里之前的bdxes处理一下就可以了
  //-----------------------------------------------------------
  g_dist_boxes_pub.publish(g_projector.dist_boxes_);
  g_jsk_boxes_pub.publish(g_projector.jsk_boxes_);
  g_jsk_box_pub.publish(g_projector.jsk_box_);
  g_lidar_obstacles_pub.publish(g_projector.lidar_obstacles_);
  g_projector.lidar_obstacles_.obstacles.clear();
  g_projector.dist_boxes_.bounding_boxes.clear();
  g_projector.jsk_boxes_.boxes.clear();

  //-----------------------------------------------------------
  sensor_msgs::Image msg_image;
  cv_bridge::CvImage img_bridge;
  img_bridge = cv_bridge::CvImage(std_msgs::Header(),
                                  "bgr8", g_receive_image);
  img_bridge.toImageMsg(msg_image);
  g_image_pub.publish(msg_image);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "one_line_lidar_camera_fusion_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  std::string config_path;

  n.getParam("config_path", config_path);
  g_projector.LoadRotationMatrix(config_path);
  g_projector.LoadTransformVector(config_path);

  image_transport::ImageTransport it(n);

  image_transport::Subscriber sub_custom_msg;
  sub_custom_msg = it.subscribe("/image_view/output", 1, VideoCallback);

  ros::Subscriber scan_sub;
  scan_sub = n.subscribe<sensor_msgs::LaserScan> ("/scan", 1, ScanCallback);

  ros::Subscriber detection_msg;
  detection_msg = n.subscribe("darknet_ros/bounding_boxes", 1, BbxCallback);

  g_image_pub = it.advertise("/image_with_dist", 1);
  g_dist_boxes_pub = n.advertise<one_line_lidar_camera_fusion
                ::BoundingBoxesWithDist>("/bounding_boxes_with_dist", 1);
  g_jsk_boxes_pub =
    n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/jsk_boxes", 5);
  g_jsk_box_pub = n.advertise<jsk_recognition_msgs::BoundingBox>("/jsk_box", 5);
  g_lidar_obstacles_pub =
    n.advertise<perception_msgs::LidarObstacles>("/perception_msgs", 5);
  ros::spin();
  return 0;
}
