#include <ros/ros.h>
#include <matplotlibcpp.h>
#include "config.h"
#include "map.h"
#include "search.h"
#include "smooth.h"
#include "resample.h"
#include "mpcc.h"


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
  Resample resample;
  Mpcc mpcc;
  map.GenerateMap();
  search.SphereSearch(map);
  smooth.Fem(search);
  resample.FitResample(smooth);
  std::vector<double> plot_xc(map.center_point_x_.data(), map.center_point_x_.data() + map.center_point_x_.size());
  std::vector<double> plot_yc(map.center_point_y_.data(), map.center_point_y_.data() + map.center_point_y_.size());
  std::vector<double> plot_xo(map.outer_point_x_.data(), map.outer_point_x_.data() + map.outer_point_x_.size());
  std::vector<double> plot_yo(map.outer_point_y_.data(), map.outer_point_y_.data() + map.outer_point_y_.size());
  std::vector<double> plot_xi(map.inner_point_x_.data(), map.inner_point_x_.data() + map.inner_point_x_.size());
  std::vector<double> plot_yi(map.inner_point_y_.data(), map.inner_point_y_.data() + map.inner_point_y_.size());
  std::vector<double> plot_x_resample(resample.spline.path_data_.X.data(), resample.spline.path_data_.X.data() + resample.spline.path_data_.X.size());
  std::vector<double> plot_y_resample(resample.spline.path_data_.Y.data(), resample.spline.path_data_.Y.data() + resample.spline.path_data_.Y.size());
  std::vector<double> x_history, y_history;
  for (int i = 0; i < 100; i++) {
    
    Eigen::SparseMatrix<double> x0(mpcc.state_dim_, 1);
    if (i == 0) {
      for (int k = 0; k < mpcc.state_dim_; k++) {
        x0.coeffRef(k, 0) = mpcc.stage[0].state[k];
      }
    } else {
      x0 = mpcc.statePredict.col(0);
    }
    
    // std::cout << i << std::endl;
    mpcc.CalculateCost(resample, x0);
    mpcc.SolveQp(x0);
    x_history.emplace_back(x0.coeffRef(0, 0));
    y_history.emplace_back(x0.coeffRef(2, 0));
  }
  std::vector<double> x_horizon, y_horizon;
  for (int i=0;i<10;i++){
    x_horizon.emplace_back(mpcc.statePredict.coeffRef(0, i));
    y_horizon.emplace_back(mpcc.statePredict.coeffRef(2, i));
  }


  plt::figure(1);
  plt::plot(plot_xc,plot_yc,"r--");
  plt::plot(plot_xo,plot_yo,"k");
  plt::plot(plot_xi,plot_yi,"k");
  // plt::named_plot("search", search.result_x, search.result_y,"p--");
  // plt::named_plot("smooth", smooth.result_x, smooth.result_y,"y.-");
  plt::named_plot("fit and resample", plot_x_resample, plot_y_resample,"m--");
  plt::named_plot("horizon", x_horizon, y_horizon,"p--");
  plt::named_plot("history", x_history, y_history,"r");
  plt::legend();
  plt::show();



  // while (ros::ok()) {
  //   ros::spinOnce();
  //   mission.process();
  //   rate.sleep();
  // }
  return 0;
}