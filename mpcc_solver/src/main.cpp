#include <ros/ros.h>
#include <matplotlibcpp.h>
#include "config.h"
#include "map.h"
#include "search.h"
#include "smooth.h"


namespace plt = matplotlibcpp;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpcc_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  ros::Rate rate(100);

  // State state;
  // Input input;
  
  Map map;
  Search search;
  Smooth smooth;
  map.GenerateMap();
  search.SphereSearch(map);
  smooth.Fem(search);

  std::vector<double> plot_xc(map.center_point_x_.data(), map.center_point_x_.data() + map.center_point_x_.size());
  std::vector<double> plot_yc(map.center_point_y_.data(), map.center_point_y_.data() + map.center_point_y_.size());
  std::vector<double> plot_xo(map.outer_point_x_.data(), map.outer_point_x_.data() + map.outer_point_x_.size());
  std::vector<double> plot_yo(map.outer_point_y_.data(), map.outer_point_y_.data() + map.outer_point_y_.size());
  std::vector<double> plot_xi(map.inner_point_x_.data(), map.inner_point_x_.data() + map.inner_point_x_.size());
  std::vector<double> plot_yi(map.inner_point_y_.data(), map.inner_point_y_.data() + map.inner_point_y_.size());
  plt::figure(1);
  plt::plot(plot_xc,plot_yc,"r--");
  plt::plot(plot_xo,plot_yo,"b");
  plt::plot(plot_xi,plot_yi,"b");
  plt::plot(search.result_x, search.result_y,"p--");
  plt::plot(smooth.result_x, smooth.result_y,"y.-");
  // plt::plot(search.clutter_inner_x, search.clutter_inner_y,"p--");
  plt::show();



  // while (ros::ok()) {
  //   ros::spinOnce();
  //   mission.process();
  //   rate.sleep();
  // }
  return 0;
}