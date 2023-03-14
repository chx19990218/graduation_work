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
  map.GenerateMap();
  search.SphereSearch(map);
  smooth.Fem(search);
  resample.FitResample(smooth);

  mpcc.state.coeffRef(0, 0) = 0.5;
  mpcc.state.coeffRef(1, 0) = 0.0;
  mpcc.state.coeffRef(2, 0) = 1.8;
  mpcc.state.coeffRef(3, 0) = 1.0;
  obstacle.Update(resample, map, mpcc);
  
  
  int cnt = 1;
  mpcc.Init(resample);
  for (int i = 0; i < cnt; i++) { 
    if (i % 10 == 0){
      std::cout<<"sim times:"<<i<<std::endl;
    } 
    auto start_time = ros::Time::now();
    mpcc.SolveQp(resample, map);
    auto end_time = ros::Time::now();
    // std::cout << (end_time - start_time).toSec() << std::endl;
    mpcc.x_history.emplace_back(mpcc.state.coeffRef(0, 0));
    mpcc.y_history.emplace_back(mpcc.state.coeffRef(2, 0));

  }
  
  for (int i=0; i<mpcc.horizon; i++){
    mpcc.x_horizon.emplace_back(mpcc.statePredict.coeffRef(0, i));
    mpcc.y_horizon.emplace_back(mpcc.statePredict.coeffRef(2, i));
  }

  plot.plot(map, search, smooth, resample, mpcc, obstacle);



  // while (ros::ok()) {
  //   ros::spinOnce();
  //   mission.process();
  //   rate.sleep();
  // }
  return 0;
}