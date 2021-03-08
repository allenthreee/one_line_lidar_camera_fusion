// Copyright 2021.02.04 <YWY>
#include <vector>
#ifndef SRC_MY_PCL_SRC_MY_STRUCTURE_H_
#define SRC_MY_PCL_SRC_MY_STRUCTURE_H_

// boundingboxes with dist
#include "one_line_lidar_camera_fusion/BoundingBoxesWithDist.h"
#include "one_line_lidar_camera_fusion/BoundingBoxWithDist.h"
struct MyPoint{
    int id;
    std::vector<float> x_coor;
    std::vector<float> y_coor;
    std::vector<float> dis;
    std::vector<float> angle;
};

struct MyRect{
  int num_;
  int id_;
  float dis_;
  float angle_;
};

struct PixelCoor{
  int pixel_x;
  int pixel_y;
  double pixel_z;
};

struct ThreeDBoundingBox{
  float three_d_box_x;
  float three_d_box_y;
};

#endif  // SRC_ONE_LINE_LIDAR_CAMERA_FUSION_SRC_MY_STRUCTURE_H_
