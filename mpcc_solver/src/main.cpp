#include <ros/ros.h>
#include <matplotlibcpp.h>
#include "config.h"
#include "map.h"
#include "search.h"
#include "smooth.h"
#include "resample.h"
#include "mpcc.h"
#include "plot.h"
#include "generate_map_test.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <quadrotor_msgs/PositionCommand.h>

void odom_callback(const nav_msgs::Odometry& odom);
void mpc_callback(const ros::TimerEvent& event);

Eigen::SparseMatrix<double> state(5, 1);
ros::Time tOdom;
nav_msgs::Path refTraj_msg;

int main(int argc, char** argv) {
  ros::init(argc, argv, "mpcc_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  ros::Rate rate(50);

  ros::Publisher drone_pub, global_pub, predict_pub, theta_predict_pub, cmd_pub, refer_pub;
  ros::Subscriber sub_odom = n.subscribe("odom", 100, odom_callback, ros::TransportHints().tcpNoDelay());
  refer_pub = n.advertise<nav_msgs::Path>("refer_path", 1);
  cmd_pub = n.advertise<quadrotor_msgs::PositionCommand>("position_cmd",1);
  drone_pub = n.advertise<visualization_msgs::Marker>("drone_pose", 1);
  global_pub = n.advertise<visualization_msgs::Marker>("global_pose", 1);
  predict_pub = n.advertise<nav_msgs::Path>("predict_path", 1);
  theta_predict_pub = n.advertise<nav_msgs::Path>("theta_predict_path", 1);

  
  Map map;
  Search search;
  Smooth smooth;
  Resample resample;
  Mpcc mpcc;
  Obstacle obstacle;
  Plot plot;

  // 生成pcd地图
  // GenerateMap();
  
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

  // publish参考线
  refTraj_msg.header.frame_id = "world";
  geometry_msgs::PoseStamped tmpPose;
  double theta = 0;
  for (int i = 0; i < resample.spline.path_data_.X.size(); i++) {
    tmpPose.pose.position.x = resample.spline.path_data_.X(i);
    tmpPose.pose.position.y = resample.spline.path_data_.Y(i);
    tmpPose.pose.position.z = 0.0;
    refTraj_msg.poses.push_back(tmpPose);
  }
  refer_pub.publish(refTraj_msg);

  // obstacle.Update(resample, map, mpcc, state);
  
  
  // int cnt = 1;
  mpcc.UpdateState(resample, state);
  mpcc.Init(resample, state);
  // for (int i = 0; i < cnt; i++) { 
  //   if (i % 10 == 0){
  //     std::cout<<"sim times:"<<i<<std::endl;
  //   } 
  //   auto start_time = ros::Time::now();
  //   // 更新theta
  //   mpcc.UpdateState(resample, state);
  //   // 求解mpcc
  //   mpcc.SolveQp(resample, map, state);
  //   auto end_time = ros::Time::now();
  //   std::cout << (end_time - start_time).toSec() << std::endl;

  //   // 仿真,先用mpcc预测第一帧
  //   state = mpcc.statePredict.col(0);

  //   // 更新绘图内容
  //   mpcc.UpdateResultForPlot(resample, state);
  // }
  
  

  // plot.plot(map, search, smooth, resample, mpcc, obstacle);



  while (ros::ok()) {
    ros::spinOnce();
    mpcc.UpdateState(resample, state);
    mpcc.SolveQp(resample, map, state);

    geometry_msgs::Point tmpPoint;
    geometry_msgs::Vector3 tmpVector;
    quadrotor_msgs::PositionCommand cmdMsg;
    tmpPoint.x = mpcc.stage[0].state[0];
    tmpPoint.y = mpcc.stage[0].state[2];
    tmpPoint.z = 0.0;
    cmdMsg.position = tmpPoint;
    tmpVector.x = mpcc.stage[0].state[1];
    tmpVector.y = mpcc.stage[0].state[3];
    tmpVector.z = 0.0;
    cmdMsg.velocity = tmpVector;
    tmpVector.x = mpcc.inputPredict.coeffRef(0, 0);
    tmpVector.y = mpcc.inputPredict.coeffRef(1, 0);
    tmpVector.z = 0.0;
    cmdMsg.acceleration = tmpVector;
    cmdMsg.header.stamp = ros::Time::now();
    rate.sleep();
    cmd_pub.publish(cmdMsg);

    nav_msgs::Path trajPred_msg;
    trajPred_msg.header.frame_id = "world";
    trajPred_msg.poses.resize(mpcc.horizon);
    geometry_msgs::PoseStamped tmpPose;
    for (int i = 0; i < mpcc.horizon; i++) {
      tmpPose.pose.position.x = mpcc.statePredict.coeffRef(0, i);
      tmpPose.pose.position.y = mpcc.statePredict.coeffRef(2, i);
      tmpPose.pose.position.z = 0.0;
      trajPred_msg.poses[i] = tmpPose;
    }
    predict_pub.publish(trajPred_msg);

    nav_msgs::Path theta_trajPred_msg;
    theta_trajPred_msg.header.frame_id = "world";
    theta_trajPred_msg.poses.resize(mpcc.horizon);
    for (int i = 0; i < mpcc.horizon; i++) {
      auto pos_theta = resample.spline.getPostion(mpcc.statePredict.coeffRef(4, i));
      tmpPose.pose.position.x = pos_theta(0);
      tmpPose.pose.position.y = pos_theta(1);
      tmpPose.pose.position.z = 0.0;
      theta_trajPred_msg.poses[i] = tmpPose;
    }
    theta_predict_pub.publish(theta_trajPred_msg);
  }
  return 0;
}

void odom_callback(const nav_msgs::Odometry& odom){
  // TODO odom valid
  // odomT = (odom.header.stamp - tOdom).toSec();
  // tOdom = odom.header.stamp;

  state.coeffRef(0, 0) = odom.pose.pose.position.x;
  state.coeffRef(1, 0) = odom.twist.twist.linear.x;
  state.coeffRef(2, 0) = odom.pose.pose.position.y;
  state.coeffRef(3, 0) = odom.twist.twist.linear.y;
}
// void mpc_callback(const ros::TimerEvent& event){
  // refTraj_msg.header.frame_id = "world";
  // geometry_msgs::PoseStamped tmpPose;
  // double theta = 0;
  // for (i = 0; i < resample.path_data_.size(); i++) {
  //   tmpPose.pose.position.x = resample.path_data_.x(i);
  //   tmpPose.pose.position.y = resample.path_data_.y(i);
  //   tmpPose.pose.position.z = 0.0;
  //   refTraj_msg.poses.push_back(tmpPose);
  // }
  // ros::Timer timer_mpc = nodeHandle.createTimer(ros::Duration(mpcT), mpc_callback);
    
  // refer_pub.publish(simSolver.displayPtr->refTraj_msg);
//     global_pub.publish(simSolver.displayPtr->theta_msg);
//     drone_pub.publish(simSolver.displayPtr->drone_msg);
//     vis_polytope_pub.publish(simSolver.displayPtr->corridor_array_msg);
//     simSolver.displayPtr->pubTunnels(flight_tunnel_pub);
//     predict_pub.publish(simSolver.displayPtr->trajPred_msg);
// }