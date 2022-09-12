#include <ros/ros.h>
#include <matplotlibcpp.h>
#include "config.h"
#include "map.h"
#include "search.h"
#include "smooth.h"
#include "resample.h"
#include "mpcc.h"
#include "plot.h"


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
  Plot plot;
  map.GenerateMap();
  search.SphereSearch(map);
  smooth.Fem(search);
  resample.FitResample(smooth);
  
  std::vector<double> x_history, y_history;
  int cnt = 10;
  for (int i = 0; i < cnt; i++) {
    
    Eigen::SparseMatrix<double> x0(mpcc.state_dim_, 1);
    if (i == 0) {
      for (int k = 0; k < mpcc.state_dim_; k++) {
        x0.coeffRef(k, 0) = mpcc.stage[0].state[k];
      }
    } else {
      x0 = mpcc.statePredict.col(0);
    }
    double now_theta = resample.spline.porjectOnSpline(x0.coeffRef(0, 0), x0.coeffRef(2, 0));
    x0.coeffRef(6, 0) = now_theta;
    
    // std::cout << i << std::endl;
    // auto start_time = ros::Time::now();

    mpcc.CalculateCost(resample, x0);
    mpcc.SolveQp(x0, resample, map);
    x_history.emplace_back(x0.coeffRef(0, 0));
    y_history.emplace_back(x0.coeffRef(2, 0));
    // auto end_time = ros::Time::now();
    // std::cout<<(end_time - start_time).toSec()<<std::endl;
  }
  std::vector<double> x_horizon, y_horizon;
  for (int i=0; i<10; i++){
    x_horizon.emplace_back(mpcc.statePredict.coeffRef(0, i));
    y_horizon.emplace_back(mpcc.statePredict.coeffRef(2, i));
  }

  plot.plot(map, search, smooth, resample, mpcc);



  // while (ros::ok()) {
  //   ros::spinOnce();
  //   mission.process();
  //   rate.sleep();
  // }
  return 0;
}