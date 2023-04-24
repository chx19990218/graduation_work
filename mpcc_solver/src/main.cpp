// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include <ros/ros.h>
#include <matplotlibcpp.h>
#include "config.h"
#include "map.h"
#include "search.h"
#include "smooth.h"
#include "resample.h"
#include "mpcc.h"
#include "plot.h"
#include "log.h"
#include "generate_map_test.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>

void odom_callback(const nav_msgs::Odometry& odom);
void optitrack_callback(const geometry_msgs::PoseStamped& odom);
void obs_callback(const nav_msgs::Path& path);
void publish_topic(Mpcc& mpcc, const Resample& resample, const Config& config);

// 动捕卡尔曼速度
void InitOdometry(const geometry_msgs::PoseStamped& odom);
void PredictOdometry(double delta);
void UpdateOdometry(const geometry_msgs::PoseStamped& odom);

Eigen::VectorXd x_p;
Eigen::MatrixXd P_p, Q_p, F_p, H_p, R_p;

Eigen::SparseMatrix<double> state(5, 1);
ros::Time tOdom;
ros::Publisher drone_pub, theta_pub, predict_pub,
  theta_predict_pub, cmd_pub, px4_pub, refer_pub;

geometry_msgs::Point tmpPoint;
geometry_msgs::Vector3 tmpVector;
nav_msgs::Path trajPred_msg;
nav_msgs::Path theta_trajPred_msg;
geometry_msgs::PoseStamped tmpPose;
geometry_msgs::Point pt;
visualization_msgs::Marker drone_msg;
visualization_msgs::Marker theta_msg;
nav_msgs::Path ego_path, obs_path;
ros::Time ego_path_time, obs_path_time;

ros::Time last_odom_time;
ros::Time last_nokov_time;
nav_msgs::Odometry log_odom;
bool nokov_init_flag = false;

bool simulation_flag;

int main(int argc, char** argv) {
  ros::init(argc, argv, "mpcc_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  Config config(nh);
  simulation_flag = config.simulation_flag;
  ros::Subscriber sub_odom = n.subscribe("odom", 100, odom_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber optitrack_odom = n.subscribe("/mavros/vision_pose/pose", 100, optitrack_callback,
    ros::TransportHints().tcpNoDelay());
  std::string obs_topic = "/drone" + std::to_string(1 - config.group_index) + "/predict_path";
  ros::Subscriber obs_sub = n.subscribe(obs_topic, 100, obs_callback, ros::TransportHints().tcpNoDelay());
  refer_pub = n.advertise<nav_msgs::Path>("refer_path", 1);
  cmd_pub = n.advertise<quadrotor_msgs::PositionCommand>("position_cmd",1);
  drone_pub = n.advertise<visualization_msgs::Marker>("drone_pose", 1);
  theta_pub = n.advertise<visualization_msgs::Marker>("theta_pose", 1);
  predict_pub = n.advertise<nav_msgs::Path>("predict_path", 1);
  theta_predict_pub = n.advertise<nav_msgs::Path>("theta_predict_path", 1);
  px4_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);

  theta_trajPred_msg.header.frame_id = "world";
  trajPred_msg.header.frame_id = "world";
  drone_msg.header.frame_id = "world";
  drone_msg.type = visualization_msgs::Marker::ARROW;
  drone_msg.action = visualization_msgs::Marker::ADD;
  drone_msg.scale.x = 0.06;
  drone_msg.scale.y = 0.1;
  drone_msg.scale.z = 0;
  drone_msg.color.a = 1;
  drone_msg.color.r = 1;
  drone_msg.color.g = 0;
  drone_msg.color.b = 0;
  drone_msg.pose.orientation.w = 1;
  theta_msg = drone_msg;
  theta_msg.color.r = 0;
  theta_msg.color.g = 1;

  Map map;
  Search search;
  Smooth smooth;
  Resample resample;
  Obstacle obstacle;
  Plot plot;
  Mpcc mpcc(config);
  // Dmpc dmpc;
  ros::Rate rate(config.ctrl_rate);

  // 生成pcd地图，覆盖掉原始pcd
  if (config.generate_pcd_map_flag) {
    GenerateMap(config, mpcc);
  }
  
  // 1. gernerate map
  auto generate_map_start_time = ros::Time::now();
  map.GenerateMap();
  auto generate_map_end_time = ros::Time::now();
  std::cout << "1. generate map finished in "
    << (generate_map_end_time - generate_map_start_time).toSec() << "s" << std::endl;
  Log log(map); 

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
  
  if (config.group_index == 1) {
    // reverse(smooth.result_x.begin(), smooth.result_x.end());
    // reverse(smooth.result_y.begin(), smooth.result_y.end());
  }

  // 4. fit and resample
  auto resample_start_time = ros::Time::now();
  resample.FitResample(smooth, config);
  auto resample_end_time = ros::Time::now();
  std::cout << "4. resample finished in "
    << (resample_end_time - resample_start_time).toSec() << "s" << std::endl;

  // publish rviz 参考线
  nav_msgs::Path refTraj_msg;
  refTraj_msg.header.frame_id = "world";
  double theta = 0;
  for (int i = 0; i < resample.spline.path_data_.X.size(); i++) {
    tmpPose.pose.position.x = resample.spline.path_data_.X(i);
    tmpPose.pose.position.y = resample.spline.path_data_.Y(i);
    tmpPose.pose.position.z = config.hover_height;
    refTraj_msg.poses.push_back(tmpPose);
  }

  ros::spinOnce();

  if (config.enable_dp_flag) {
    ros::Time dp_start_time = ros::Time::now();
    obstacle.Update(resample, map, mpcc, state, config);
    double dp_time = (ros::Time::now() - dp_start_time).toSec();
    std::cout << "dp time : " << dp_time << std::endl;
  }

  int i = 0;
  while (ros::ok()) {
    ros::Time mpcc_start_time = ros::Time::now();
    i++;
    refer_pub.publish(refTraj_msg);
    ros::spinOnce();
    rate.sleep();
    mpcc.UpdateState(resample, state);
    
    // 检查是否在走廊范围内
    bool in_corridor_range = mpcc.InCorridorRange(map, state.coeffRef(0, 0), state.coeffRef(2, 0));
    if (!in_corridor_range) {
      // 不在范围内立即停用mpcc
      mpcc.mpcc_valid_flag_ = false;
    } else {
      // 在范围内, 如果离参考线不远, 启用mpcc
      // 一旦启用，只有出范围 和 连续五帧无解 才会关掉
      auto pos_theta = resample.spline.getPostion(state.coeffRef(4, 0));
      bool long_dist_with_refer_flag = std::sqrt(std::pow(pos_theta(0) - state.coeffRef(0, 0), 2)
        + std::pow(pos_theta(1) - state.coeffRef(2, 0), 2)) > config.mpcc_valid_dist;
      if (!mpcc.mpcc_valid_flag_ && !long_dist_with_refer_flag) {
        mpcc.mpcc_valid_flag_ = true;
        mpcc.init_flag = true;
      }
    }
    // std::cout << "before:" << config.group_index << "," << in_corridor_range << "," << mpcc.output_index
    //   << "," << mpcc.init_flag << "," << mpcc.mpcc_valid_flag_ << std::endl;
    // ros::Time qp_start_time = ros::Time::now();
    mpcc.SolveQp(resample, map, config, state, ego_path, obs_path);

    // std::cout << "after:" << config.group_index << "," << in_corridor_range << "," << mpcc.output_index
    //   << "," << mpcc.init_flag << "," << mpcc.mpcc_valid_flag_ << std::endl;
    // double qp_time = (ros::Time::now() - qp_start_time).toSec();

    mpcc.x_history.emplace_back(state.coeffRef(0, 0));
    mpcc.y_history.emplace_back(state.coeffRef(2, 0));

    publish_topic(mpcc, resample, config);

    // record log
    if (config.log_flag) {
      log.Record(mpcc.cmdMsg, log_odom);
    }
    
    double mpcc_time = (ros::Time::now() - mpcc_start_time).toSec();
    if (i % 10 == 0) {
      // std::cout << "mpcc time  : " << mpcc_time << std::endl;
      // std::cout << "qp time : " << qp_time << std::endl;
    }
  }
  // std::cout << "max_a : " << mpcc.max_cmd_a << std::endl;
  if (config.simulation_flag) {
    plot.plot(map, search, smooth, resample, mpcc, obstacle);
  }
  return 0;
}

void odom_callback(const nav_msgs::Odometry& odom){
  if (simulation_flag) {
    last_odom_time = ros::Time::now();
    log_odom = odom;
    state.coeffRef(0, 0) = odom.pose.pose.position.x;
    state.coeffRef(1, 0) = odom.twist.twist.linear.x;
    state.coeffRef(2, 0) = odom.pose.pose.position.y;
    state.coeffRef(3, 0) = odom.twist.twist.linear.y;
  }
}

void optitrack_callback(const geometry_msgs::PoseStamped& odom) {
  if (!simulation_flag) {
    last_odom_time = ros::Time::now();
    if (!nokov_init_flag) {
      InitOdometry(odom);
      nokov_init_flag = true;
    }
    ros::Time now_nokov_time = ros::Time::now();
    double delta = (now_nokov_time - last_nokov_time).toSec();
    PredictOdometry(delta);
    UpdateOdometry(odom);

    state.coeffRef(0, 0) = x_p(0);
    state.coeffRef(1, 0) = x_p(3);
    state.coeffRef(2, 0) = x_p(1);
    state.coeffRef(3, 0) = x_p(4);

    nav_msgs::Odometry odom_temp;
    odom_temp.pose.pose.position.x = x_p(0);
    odom_temp.pose.pose.position.y = x_p(1);
    odom_temp.pose.pose.position.z = x_p(2);
    odom_temp.twist.twist.linear.x = x_p(3);
    odom_temp.twist.twist.linear.y = x_p(4);
    odom_temp.twist.twist.linear.z = x_p(5);
    log_odom = odom_temp;

    last_nokov_time = ros::Time::now();
  }
}

void obs_callback(const nav_msgs::Path& path) {
  obs_path = path;
  obs_path_time = ros::Time::now();
}

void publish_topic(Mpcc& mpcc, const Resample& resample, const Config& config) {
  // 控制指令
  cmd_pub.publish(mpcc.cmdMsg);

  mavros_msgs::PositionTarget px4_msg;
  px4_msg.coordinate_frame = 1;
  px4_msg.position.x = mpcc.cmdMsg.position.x;  
  px4_msg.position.y = mpcc.cmdMsg.position.y;
  px4_msg.position.z = mpcc.cmdMsg.position.z;   
  px4_msg.velocity.x = mpcc.cmdMsg.velocity.x;
  px4_msg.velocity.y = mpcc.cmdMsg.velocity.y;
  px4_msg.velocity.z = mpcc.cmdMsg.velocity.z;
  px4_msg.acceleration_or_force.x = mpcc.cmdMsg.acceleration.x;          			
  px4_msg.acceleration_or_force.y = mpcc.cmdMsg.acceleration.y;
  px4_msg.acceleration_or_force.z = mpcc.cmdMsg.acceleration.z;
  px4_pub.publish(px4_msg);

  // if (config.simulation_flag) {
    // rviz 无人机箭头
    drone_msg.points.clear();
    pt.x = state.coeffRef(0,0);
    pt.y = state.coeffRef(2,0);
    pt.z = config.hover_height;
    drone_msg.points.push_back(pt);
    pt.x += state.coeffRef(1,0) / 2.0;
    pt.y += state.coeffRef(3,0) / 2.0;
    pt.z += 0.0;
    drone_msg.points.push_back(pt);
    drone_pub.publish(drone_msg);

    // mpcc无效时，向state的theta点走
    if (mpcc.mpcc_valid_flag_) {
      // rviz 预测轨迹
      trajPred_msg.poses.resize(mpcc.horizon);
      for (int i = 0; i < mpcc.horizon; i++) {
        tmpPose.pose.position.x = mpcc.statePredict.coeffRef(0, i);
        tmpPose.pose.position.y = mpcc.statePredict.coeffRef(2, i);
        tmpPose.pose.position.z = config.hover_height;

        double psi = atan2(mpcc.statePredict.coeffRef(3, i), mpcc.statePredict.coeffRef(1, i));
        Eigen::Quaterniond q = Eigen::AngleAxisd(psi, ::Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(0.0, ::Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.0, ::Eigen::Vector3d::UnitX());
        tmpPose.pose.orientation.x = q.x();
        tmpPose.pose.orientation.y = q.y();
        tmpPose.pose.orientation.z = q.z();
        tmpPose.pose.orientation.w = q.w();
        trajPred_msg.poses[i] = tmpPose;
      }
      predict_pub.publish(trajPred_msg);

      // rviz theta预测轨迹
      theta_trajPred_msg.poses.resize(mpcc.horizon);
      for (int i = 0; i < mpcc.horizon; i++) {
        auto pos_theta = resample.spline.getPostion(mpcc.statePredict.coeffRef(4, i));
        tmpPose.pose.position.x = pos_theta(0);
        tmpPose.pose.position.y = pos_theta(1);
        tmpPose.pose.position.z = config.hover_height + 0.1;
        theta_trajPred_msg.poses[i] = tmpPose;
      }
      theta_predict_pub.publish(theta_trajPred_msg);

      // rviz theta箭头
      theta_msg.points.clear();
      auto pos = resample.spline.getPostion(state.coeffRef(4,0));
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = config.hover_height;
      theta_msg.points.push_back(pt);
      auto vel = resample.spline.getDerivative(state.coeffRef(4,0));
      vel(0) /= sqrt(vel(0) * vel(0) + vel(1) * vel(1));
      vel(1) /= sqrt(vel(0) * vel(0) + vel(1) * vel(1));
      pt.x += vel(0);
      pt.y += vel(1);
      pt.z += 0.0;
      theta_msg.points.push_back(pt);
      theta_pub.publish(theta_msg);
    } else {
      // rviz theta箭头
      theta_msg.points.clear();
      pt.x = state.coeffRef(0,0);
      pt.y = state.coeffRef(2,0);
      pt.z = config.hover_height;
      theta_msg.points.push_back(pt);
      auto pos = resample.spline.getPostion(state.coeffRef(4,0));
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = config.hover_height;
      theta_msg.points.push_back(pt);
      theta_pub.publish(theta_msg);
    }
  // }
}

void InitOdometry(const geometry_msgs::PoseStamped& odom) {
  x_p = Eigen::VectorXd(6);
  x_p << odom.pose.position.x, odom.pose.position.y, odom.pose.position.z, 0.0, 0.0, 0.0;
  P_p = Eigen::MatrixXd::Identity(6, 6);
  P_p(0, 0) = 0.1 * 0.1;
  P_p(1, 1) = 0.1 * 0.1;
  P_p(2, 2) = 0.1 * 0.1;
  P_p(3, 3) = 0.1 * 0.1;
  P_p(4, 4) = 0.1 * 0.1;
  P_p(5, 5) = 0.1 * 0.1;
  Q_p = Eigen::MatrixXd::Identity(6, 6);
  Q_p(0, 0) = 0.0005 * 0.0005;
  Q_p(1, 1) = 0.0005 * 0.0005;
  Q_p(2, 2) = 0.0005 * 0.0005;
  Q_p(3, 3) = 0.005 * 0.005;
  Q_p(4, 4) = 0.005 * 0.005;
  Q_p(5, 5) = 0.005 * 0.005;
  F_p = Eigen::MatrixXd::Identity(6, 6);
  H_p = Eigen::Matrix<double, 3, 6>();
  H_p << 1.0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0;
  R_p = Eigen::MatrixXd::Identity(3, 3);
  R_p(0, 0) = 0.01 * 0.01;
  R_p(1, 1) = 0.01 * 0.01;
  R_p(2, 2) = 0.01 * 0.01;
}

void PredictOdometry(double delta) {
  F_p(0, 0) = 1.0;
  F_p(1, 1) = 1.0;
  F_p(2, 2) = 1.0;
  F_p(3, 3) = 1.0;
  F_p(4, 4) = 1.0;
  F_p(5, 5) = 1.0;
  F_p(0, 3) = delta;
  F_p(1, 4) = delta;
  F_p(2, 5) = delta;
  x_p = F_p * x_p;
  P_p = F_p * P_p * F_p.transpose() + Q_p;
}

void UpdateOdometry(const geometry_msgs::PoseStamped& odom) {
  Eigen::Matrix<double, 3, 1> z_p;
  z_p << odom.pose.position.x, odom.pose.position.y, odom.pose.position.z;
  Eigen::MatrixXd y_p = z_p - H_p * x_p;
  Eigen::MatrixXd k_p = P_p * H_p.transpose() * (H_p * P_p * H_p.transpose() + R_p).inverse();
  x_p = x_p + k_p * y_p;
  P_p = (Eigen::MatrixXd::Identity(6, 6) - k_p * H_p) * P_p;
}