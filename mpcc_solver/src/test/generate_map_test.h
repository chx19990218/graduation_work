// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include <fstream>
#include <iostream>
#include "map.h"
#include "mpcc.h"

void GenerateMap(const Config& config, const Mpcc& mpcc) {
  Map map;
  map.GenerateMap();
  std::ofstream flightlog;
  std::string path = "/home/chx/graduation_work/src/mpcc_solver/src/test/map.txt";
  flightlog.open(path, std::ios::out);
  flightlog << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
  flightlog << "VERSION 0.7" << std::endl;
  flightlog << "FIELDS x y z" << std::endl;
  flightlog << "SIZE 4 4 4" << std::endl;
  flightlog << "TYPE F F F" << std::endl;
  flightlog << "COUNT 1 1 1" << std::endl;
  double interval = 0.1;
  int height = 20;
  int dot_count = 0;
  for (int i = 0; i < map.center_point_x_.size() - 1; i++) {
    double outer_next_x = map.outer_point_x_[i + 1];
    double outer_next_y = map.outer_point_y_[i + 1];
    double outer_now_x = map.outer_point_x_[i];
    double outer_now_y = map.outer_point_y_[i];
    double outer_dist = std::sqrt(std::pow(outer_next_x - outer_now_x, 2) + std::pow(outer_next_y - outer_now_y, 2));
    int outer_num = static_cast<int>(outer_dist / interval);
    dot_count += outer_num;

    double inner_next_x = map.inner_point_x_[i + 1];
    double inner_next_y = map.inner_point_y_[i + 1];
    double inner_now_x = map.inner_point_x_[i];
    double inner_now_y = map.inner_point_y_[i];
    double inner_dist = std::sqrt(std::pow(inner_next_x - inner_now_x, 2) + std::pow(inner_next_y - inner_now_y, 2));
    int inner_num = static_cast<int>(inner_dist / interval);
    dot_count += inner_num;
  }

  // double x_l = std::min(mpcc.obstacle_pos_[0][0], mpcc.obstacle_pos_[2][0]);
  // double x_r = std::max(mpcc.obstacle_pos_[0][0], mpcc.obstacle_pos_[2][0]);
  // double y_l = std::min(mpcc.obstacle_pos_[0][1], mpcc.obstacle_pos_[2][1]);
  // double y_r = std::max(mpcc.obstacle_pos_[0][1], mpcc.obstacle_pos_[2][1]);
  // for (double i = x_l; i <= x_r; i += interval) {
  //   for (double j = y_l; j <= y_r; j += interval) {
  //     dot_count++;
  //   }
  // }
  dot_count *= height;
  // 边界四个点加上
  dot_count += 4;
  
  flightlog << "WIDTH " << dot_count << std::endl;
  flightlog << "HEIGHT 1" << std::endl;
  flightlog << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
  flightlog << "POINTS " << dot_count << std::endl;
  flightlog << "DATA ascii" << std::endl;
  for (int i = 0; i < map.center_point_x_.size() - 1; i++) {
    double outer_next_x = map.outer_point_x_[i + 1];
    double outer_next_y = map.outer_point_y_[i + 1];
    double outer_now_x = map.outer_point_x_[i];
    double outer_now_y = map.outer_point_y_[i];
    double outer_dist = std::sqrt(std::pow(outer_next_x - outer_now_x, 2) + std::pow(outer_next_y - outer_now_y, 2));
    int outer_num = static_cast<int>(outer_dist / interval);
    double outer_x_interval = (outer_next_x - outer_now_x) / static_cast<double>(outer_num);
    double outer_y_interval = (outer_next_y - outer_now_y) / static_cast<double>(outer_num);

    double inner_next_x = map.inner_point_x_[i + 1];
    double inner_next_y = map.inner_point_y_[i + 1];
    double inner_now_x = map.inner_point_x_[i];
    double inner_now_y = map.inner_point_y_[i];
    double inner_dist = std::sqrt(std::pow(inner_next_x - inner_now_x, 2) + std::pow(inner_next_y - inner_now_y, 2));
    int inner_num = static_cast<int>(inner_dist / interval);
    double inner_x_interval = (inner_next_x - inner_now_x) / static_cast<double>(inner_num);
    double inner_y_interval = (inner_next_y - inner_now_y) / static_cast<double>(inner_num);

    for (int j = 0; j < outer_num; j++) {
      for (int k = 0; k < height; k++) {
        flightlog << outer_now_x + j * outer_x_interval << " "
          << outer_now_y + j * outer_y_interval << " " << k * 0.05 << std::endl;
      }
    }

    for (int j = 0; j < inner_num; j++) {
      for (int k = 0; k < height; k++) {
        flightlog << inner_now_x + j * inner_x_interval << " "
          << inner_now_y + j * inner_y_interval << " " << k * 0.05 << std::endl;
      }
    }
  }
  // for (double i = x_l; i <= x_r; i += interval) {
  //   for (double j = y_l; j <= y_r; j += interval) {
  //     for (int k = 0; k < height; k++) {
  //       flightlog << i << " " << j << " " << k * 0.05 << std::endl;
  //     }
  //   }
  // }
  flightlog << 4.5 << " " << 1.8 << " " << 1.2 << std::endl;
  flightlog << 4.5 << " " << -1.8 << " " << 1.2 << std::endl;
  flightlog << -3.0 << " " << 1.8 << " " << 1.2 << std::endl;
  flightlog << -3.0 << " " << -1.8 << " " << 1.2 << std::endl;
  flightlog.close();
}