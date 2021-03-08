// copyright 2021.02.04 <YWY>
#include "single_line_projection.h"

bool Comp(const double &a, const double &b) {
  return a < b;
}

SingleLineProjection::SingleLineProjection() {
  float three_d_box_x_ = 0;
  float three_d_box_y_ = 0;
}

void SingleLineProjection::LidarProjection2Camera(cv::Mat& raw_img, int box_num,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud,
                                 const darknet_ros_msgs::BoundingBoxes &bbxes) {
  std::vector<MyPoint> my_points(box_num);
  // cv::Mat show_image = raw_img;
  bool point_value_ = false;
  // 遍历所有点
  for (int i = 0; i < inCloud->size(); i++) {
    Eigen::Vector3f cloudpoint;
    PixelCoor pixel_coor_val_this;
    cloudpoint << inCloud->points[i].x,
    inCloud->points[i].y, inCloud->points[i].z;

    pixel_coor_val_this = CalculatePixelCoor(cloudpoint);
    int pixex = pixel_coor_val_this.pixel_x;
    int pixey = pixel_coor_val_this.pixel_y;
    double pixez = pixel_coor_val_this.pixel_z;

    double xmin, ymin, xmax, ymax;
    // 如果点在图片上,就把点画出来
    if (pixex > 2 && pixex < raw_img.size[1] && pixey > 2 &&
        pixey < raw_img.size[0]-2 && pixez > 0.0) {
      point_value_ = true;
      cv::Point circle_center(pixex, pixey);
      double dis_ = sqrt(pow(inCloud->points[i].x, 2)
      +pow(inCloud->points[i].y, 2));
      DrawCircle(raw_img, circle_center, dis_);

      // 遍历bbx，如果点在检测框内
      // in boudnding box [nn]th
      for (int n = 0; n < box_num; n++) {
        xmin = bbxes.bounding_boxes[n].xmin;
        xmax = bbxes.bounding_boxes[n].xmax;
        if (pixex > xmin && pixex < xmax) {
          if (!my_points[n].dis.size()) {   // 如果bbxes[n]里面没有点
            double tan_ = inCloud->points[i].y/inCloud->points[i].x;
            my_points[n].id = bbxes.bounding_boxes[n].id;
            my_points[n].dis.push_back(dis_);
            my_points[n].angle.push_back(tan_);
            my_points[n].x_coor.push_back(inCloud->points[i].x);
            my_points[n].y_coor.push_back(inCloud->points[i].y);
          } else {
            double tan_ = inCloud->points[i].y/inCloud->points[i].x;
            my_points[n].dis.push_back(dis_);
            my_points[n].angle.push_back(tan_);
            my_points[n].x_coor.push_back(inCloud->points[i].x);
            my_points[n].y_coor.push_back(inCloud->points[i].y);
          }
          break;
        }
      }
    } else {  // if else 图片不在点上，continue
        continue;
    }
  }



  if (point_value_) {   // 如果图片上有点
  // 遍历n个bbx，计算他们的距离、三维空间中盒子的位置
      for (int i = 0; i < box_num; i++) {
        double three_d_box_x;
        float three_d_box_y;
        one_line_lidar_camera_fusion::BoundingBoxWithDist dist_box;
        three_d_box_x = CalculateThreeDBoxX(bbxes.bounding_boxes[i],
                                       &my_points[i]);
        three_d_box_y = CalculateThreeDBoxY(bbxes.bounding_boxes[i],
                                       &my_points[i]);
        dist_box = CalculateCubeDist(bbxes.bounding_boxes[i],
                                      &my_points[i], raw_img);
        jsk_box_.pose.position.x = three_d_box_x;
        jsk_box_.pose.position.y = three_d_box_y;
        jsk_box_.pose.position.z = 0.0;
        jsk_box_.dimensions.x = jsk_box_.dimensions.y = jsk_box_.dimensions.z = 0.3;
        jsk_box_.value = 0.3;
        jsk_box_.label = 1;
        jsk_box_.header.frame_id = "laser";
        jsk_box_.pose.orientation.w = 1;
        lidar_obstacle_.header.frame_id = "laser";
        lidar_obstacle_.header.stamp = ros::Time::now();
        lidar_obstacle_.source = 0;
        lidar_obstacle_.position.x = three_d_box_x;
        lidar_obstacle_.position.y = three_d_box_y;
        lidar_obstacle_.position.z = 0.0;
        lidar_obstacle_.dimensions.x = lidar_obstacle_.dimensions.y = 0.4;
        lidar_obstacle_.dimensions.z = 1.7;
        lidar_obstacle_.minheight = 0.0;
        lidar_obstacle_.maxheight = 1.7;
        lidar_obstacle_.type = 1;

        jsk_boxes_.boxes.push_back(jsk_box_);
        dist_boxes_.bounding_boxes.push_back(dist_box);
        lidar_obstacles_.obstacles.push_back(lidar_obstacle_);
      }
    }
    jsk_boxes_.header.frame_id = "laser";
    jsk_boxes_.header.stamp = dist_boxes_.header.stamp = ros::Time::now();
    lidar_obstacles_.header.frame_id = "laser";
    lidar_obstacles_.header.stamp = ros::Time::now();
    dist_boxes_.header.frame_id = "detection_dist";
    cv::waitKey(1);
}

void SingleLineProjection::DrawCircle
(cv::Mat& raw_img, cv::Point const& circle_center, double const& point_dist) {
  double maxVal = 5.0;
  int circle_radius = 5;
  int green_dist = static_cast<int>(255 * fabs(point_dist / maxVal));
  int red_dist = 255+50-green_dist;
  int red = std::min(255, red_dist);
  int green = std::min(255, green_dist);
  // (B, G, R)
  cv::Scalar circle_color(0, green, red);

  cv::circle(raw_img, circle_center, circle_radius, circle_color, cv::FILLED);
}

PixelCoor SingleLineProjection::CalculatePixelCoor
(Eigen::Vector3f cloud_point) {
  PixelCoor pixel_coor_val;
  Eigen::Matrix3f intrinsic_eigen;
  intrinsic_eigen << 896.4223, 0, 649.2941, 0, 895.2755, 328.0513, 0, 0, 1;

  Eigen::Vector3f camerapoint;
  Eigen::Vector3f pixelpoint;

  camerapoint = world_to_cam_r_*cloud_point + world_to_cam_t_;

  pixelpoint = intrinsic_eigen*camerapoint;

  pixel_coor_val.pixel_x = pixelpoint(0)/pixelpoint(2);
  pixel_coor_val.pixel_y = pixelpoint(1)/pixelpoint(2);
  pixel_coor_val.pixel_z = pixelpoint(2);

  return pixel_coor_val;
}

// on_line_lidar_camera_fusion is a namespace created automatically by rosmsg
// 同时也是函数CalculateCubeDist的返回值
one_line_lidar_camera_fusion::BoundingBoxWithDist SingleLineProjection::CalculateCubeDist
  (const darknet_ros_msgs::BoundingBox& box, MyPoint* points, cv::Mat& img) {
  // std::cout << "points->dis.size(): " << points->dis.size() << std::endl;
  std::string dis_string;
  MyRect my_rect;
  cv::Rect raw_rect;
  one_line_lidar_camera_fusion::BoundingBoxWithDist dist_box;
  sort(points->dis.begin(), points->dis.end(), Comp);
  raw_rect.x = box.xmin;
  raw_rect.y = box.ymin;
  raw_rect.width = (box.xmax-box.xmin);
  raw_rect.height =(box.ymax-box.ymin);

  if (points->dis.size() <= 0) {
    DrawCube(img, raw_rect);
    // std::cout << "No point falls in the box" << std::endl;
  } else {
      my_rect.num_ = points->dis.size();
      my_rect.id_ = points->id;
      my_rect.dis_ = points->dis[(points->dis.size()/2)];
      // we choose the middle of the vector as its distance.
      my_rect.angle_ = points->angle[0];

      dis_string =  std::to_string(my_rect.dis_);
      dis_string = dis_string.substr(0, 4);
      std::string obj_ = box.Class;
      DrawCube(img, raw_rect, dis_string, obj_);
  }

  dist_box.probability = box.probability;
  dist_box.xmin = box.xmin;
  dist_box.ymin = box.ymin;
  dist_box.xmax = box.xmax;
  dist_box.ymax = box.ymax;
  dist_box.id = box.id;
  dist_box.Class = box.Class;
  dist_box.dist = dis_string;
  dist_box.point_counts = points->dis.size();

  return dist_box;
}

float SingleLineProjection::CalculateThreeDBoxX
  (const darknet_ros_msgs::BoundingBox& box, MyPoint* points) {
  float x;
  sort(points->x_coor.begin(), points->x_coor.end(), Comp);
  if (points->x_coor.size() <= 0) {
    std::cout << "points.x_coor is empty in this round" << std::endl;
    x = this->three_d_box_x_;
    // return;
  } else {
    std::cout << "the number of points: " << points->x_coor.size() << std::endl;
    x = points->x_coor[(points->x_coor.size()/2)];
  }
  this->three_d_box_x_ = x;
  std::cout << "x value is: " << x << std::endl;

  return x;
}


float SingleLineProjection::CalculateThreeDBoxY
  (const darknet_ros_msgs::BoundingBox& box, MyPoint* points) {
  float y;
  sort(points->y_coor.begin(), points->y_coor.end(), Comp);
  if (points->y_coor.size() <= 0) {
    y = this->three_d_box_y_;
  } else {
    y = points->y_coor[(points->y_coor.size()/2)];
  }
  this->three_d_box_y_ = y;

  return y;
}


void SingleLineProjection::DrawCube(cv::Mat& img, cv::Rect r) {
  std::vector<cv::Point2d> corners_2d;
  cv::Point2d tmp;
  cv::Point2d tmp_upper;
  tmp_upper.x = r.x + r.height;

  // tmp_upper.y = r.y - r.height * 1.7;
  tmp_upper.y = r.y - 0.5 * r.height;
  for (int i = 0; i < 8; i++) {
    switch (i) {
      case 0: {
        tmp.x = r.x;
        tmp.y = r.y;
        corners_2d.push_back(tmp);
        break;
      }
      case 1: {
        tmp.x = r.x + r.width;
        tmp.y = r.y;
        corners_2d.push_back(tmp);
        break;
      }
      case 2: {
        tmp.x = r.x;
        tmp.y = r.y + r.height;
        corners_2d.push_back(tmp);
        break;
      }
      case 3: {
        tmp.x = r.x + r.width;
        tmp.y = r.y + r.height;
        corners_2d.push_back(tmp);
        break;
      }
    }
  }
  int thickness = 2;
  line(img, corners_2d[0], corners_2d[1], cv::Scalar(197, 75, 30), thickness);
  line(img, corners_2d[0], corners_2d[2], cv::Scalar(197, 75, 30), thickness);
  line(img, corners_2d[1], corners_2d[3], cv::Scalar(197, 75, 30), thickness);
  line(img, corners_2d[2], corners_2d[3], cv::Scalar(197, 75, 30), thickness);
}

void SingleLineProjection::DrawCube(cv::Mat& img, cv::Rect r,
                   std::string dist, std::string object) {
  std::vector<cv::Point2d> corners_2d;
  cv::Point2d tmp;
  cv::Point2d tmp_upper;
  tmp_upper.x = r.x + r.height;
  // tmp_upper.y = r.y - r.height * 1.7;
  tmp_upper.y = r.y - 0.5 * r.height;
  for (int i = 0; i < 8; i++) {
    switch (i) {
      case 0: {
        tmp.x = r.x;
        tmp.y = r.y;
        corners_2d.push_back(tmp);
        break;
      }
      case 1: {
        tmp.x = r.x + r.width;
        tmp.y = r.y;
        corners_2d.push_back(tmp);
        break;
      }
      case 2: {
        tmp.x = r.x;
        tmp.y = r.y + r.height;
        corners_2d.push_back(tmp);
        break;
      }
      case 3: {
        tmp.x = r.x + r.width;
        tmp.y = r.y + r.height;
        corners_2d.push_back(tmp);
        break;
      }
    }
  }
  int thickness = 2;
  line(img, corners_2d[0], corners_2d[1], cv::Scalar(197, 75, 30), thickness);
  line(img, corners_2d[0], corners_2d[2], cv::Scalar(197, 75, 30), thickness);
  line(img, corners_2d[1], corners_2d[3], cv::Scalar(197, 75, 30), thickness);
  line(img, corners_2d[2], corners_2d[3], cv::Scalar(197, 75, 30), thickness);
  cv::putText(img, dist, cv::Point(r.x+5, r.y + 50),
                cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0, 0, 255));
  cv::putText(img, object, cv::Point(r.x+5, r.y + 100),
                cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0, 0, 255));
}

void SingleLineProjection::LoadRotationMatrix(const std::string& config_path) {
  std::ifstream R;
  R.open(config_path+"R.txt");
  Eigen::Matrix3f R_;
  int RMatrix[9]={ 0 };
  for (int i = 0; i < 9; i++) {
    R >> RMatrix[i];
    // R_ << RMatrix[i];
    // std::cout<<"RMatrix["<<i<<"] is: "<<RMatrix[i]<<std::endl;
  }
  R_ << RMatrix[0], RMatrix[1], RMatrix[2], RMatrix[3],
      RMatrix[4], RMatrix[5], RMatrix[6], RMatrix[7], RMatrix[8];
  R.close();
  // std::cout << "the R matrix is: " << std::endl << R_ << std::endl;

  std::ifstream PitchYawRoll;
  PitchYawRoll.open(config_path+"Pitch_Yaw_Roll.txt");
  float pitch, yaw, roll;
  PitchYawRoll >> pitch >> yaw >> roll;
  PitchYawRoll.close();
  pitch = pitch*acos(-1)/180;
  yaw = yaw*acos(-1)/180;
  roll = roll*acos(-1)/180;

  Eigen::Matrix3f pitch_;
  pitch_<< 1,      0,      0,
           0, cos(pitch), -sin(pitch),
           0, sin(pitch), cos(pitch);

  Eigen::Matrix3f yaw_;
  yaw_<< cos(yaw),  0,  sin(yaw),
            0,      1,       0,
        -sin(yaw),  0,  cos(yaw);

  Eigen::Matrix3f roll_;
  roll_<< cos(roll), -sin(roll), 0,
          sin(roll), cos(roll),  0,
            0,        0,         1;

  this->world_to_cam_r_<< 0, -1, 0,
                                0,  0, -1,
                                1,  0, 0;

  world_to_cam_r_ = world_to_cam_r_* pitch_* yaw_* roll_;
}

void SingleLineProjection::LoadTransformVector(const std::string& config_path) {
    std::ifstream T;
    T.open(config_path+"T.txt");
    float TMatrix[3];
    for (int i = 0; i < 3; i++) {
        T >> TMatrix[i];
    }

    world_to_cam_t_<< TMatrix[0], TMatrix[1], TMatrix[2];

    T.close();
}

