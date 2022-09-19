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

  ros::Rate rate(10);

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
  
  
  int cnt = 2;
  mpcc.Init(resample);
  for (int i = 0; i < cnt; i++) { 
    mpcc.SolveQp(resample, map);

    mpcc.x_history.emplace_back(mpcc.state.coeffRef(0, 0));
    mpcc.y_history.emplace_back(mpcc.state.coeffRef(2, 0));

  }
  
  for (int i=0; i<mpcc.horizon; i++){
    mpcc.x_horizon.emplace_back(mpcc.statePredict.coeffRef(0, i));
    mpcc.y_horizon.emplace_back(mpcc.statePredict.coeffRef(2, i));
  }

  plot.plot(map, search, smooth, resample, mpcc);



  // while (ros::ok()) {
  //   ros::spinOnce();
  //   mission.process();
  //   rate.sleep();
  // }
  return 0;
}