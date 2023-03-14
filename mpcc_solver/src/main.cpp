#include <ros/ros.h>
#include <matplotlibcpp.h>
#include "config.h"
#include "map.h"
#include "search.h"
#include "smooth.h"
#include "resample.h"
#include "mpcc.h"
#include "plot.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpcc_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  ros::Rate rate(10);

  // State state;
  // Input input;
  
  Map map;
  Search search;
  Smooth smooth;
  Resample resample;
  Mpcc mpcc;
  Obstacle obstacle;
  Plot plot;
  
  // 1. gernerate map
  auto generate_map_start_time = ros::Time::now();
  map.GenerateMap();
  auto generate_map_end_time = ros::Time::now();
  std::cout << "1. generate map finished in "
    << (generate_map_end_time - generate_map_start_time).toSec() << "s" << std::endl;

  // 2. search
  auto search_start_time = ros::Time::now();
  search.SphereSearch(map);
  auto search_end_time = ros::Time::now();
  std::cout << "2. search finished in "
    << (search_end_time - search_start_time).toSec() << "s" << std::endl;

  // 3. smooth
  auto smooth_start_time = ros::Time::now();
  smooth.Fem(search);
  auto smooth_end_time = ros::Time::now();
  std::cout << "3. smooth finished in "
    << (smooth_end_time - smooth_start_time).toSec() << "s" << std::endl;

  // 4. fit and resample
  auto resample_start_time = ros::Time::now();
  resample.FitResample(smooth);
  auto resample_end_time = ros::Time::now();
  std::cout << "4. resample finished in "
    << (resample_end_time - resample_start_time).toSec() << "s" << std::endl;


  Eigen::SparseMatrix<double> state;
  state.resize(mpcc.state_dim_, 1);
  state.coeffRef(0, 0) = 0.5;
  state.coeffRef(1, 0) = 0.0;
  state.coeffRef(2, 0) = 0.7;
  state.coeffRef(3, 0) = 0.0;
  obstacle.Update(resample, map, mpcc, state);
  
  
  int cnt = 500;
  mpcc.UpdateState(resample, state);
  mpcc.Init(resample, state);
  for (int i = 0; i < cnt; i++) { 
    if (i % 10 == 0){
      std::cout<<"sim times:"<<i<<std::endl;
    } 
    auto start_time = ros::Time::now();
    // 更新theta
    mpcc.UpdateState(resample, state);
    // 求解mpcc
    mpcc.SolveQp(resample, map, state);
    auto end_time = ros::Time::now();
    std::cout << (end_time - start_time).toSec() << std::endl;

    // 仿真,先用mpcc预测第一帧
    state = mpcc.statePredict.col(0);

    // 更新绘图内容
    mpcc.UpdateResultForPlot(resample, state);
  }
  
  

  plot.plot(map, search, smooth, resample, mpcc, obstacle);



  // while (ros::ok()) {
  //   ros::spinOnce();
  //   mission.process();
  //   rate.sleep();
  // }
  return 0;
}