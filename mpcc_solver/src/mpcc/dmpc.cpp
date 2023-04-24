// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "mpcc.h"

int Mpcc::CheckCollision(const nav_msgs::Path& ego_path, const nav_msgs::Path& obs_path) {
  int ego_size = ego_path.poses.size();
  int obs_size = obs_path.poses.size();
  double radius = uav_size + buffer;
  // 机头朝向x轴，左前开始逆时针
  std::vector<std::vector<double>> relative_pos{{radius, radius}, {-radius, radius},
                                                {-radius, -radius}, {radius, -radius}};
  for (int i = 0; i < std::min(ego_size, obs_size); i++) {
    double ego_q0 = ego_path.poses[i].pose.orientation.w;
    double ego_q1 = ego_path.poses[i].pose.orientation.x;
    double ego_q2 = ego_path.poses[i].pose.orientation.y;
    double ego_q3 = ego_path.poses[i].pose.orientation.z;
    double ego_yaw = atan2(2 * ego_q1 * ego_q2 + 2 * ego_q0 * ego_q3, -2 * ego_q2 * ego_q2 - 2 * ego_q3 * ego_q3 + 1);
    // 四个顶点
    std::vector<std::vector<double>> ego_rect(4,
      std::vector<double>{ego_path.poses[i].pose.position.x, ego_path.poses[i].pose.position.y});
    for (int i = 0; i < 4; i++) {
      ego_rect[i][0] += relative_pos[i][0] * std::cos(ego_yaw) - relative_pos[i][1] * std::sin(ego_yaw);
      ego_rect[i][1] += relative_pos[i][0] * std::sin(ego_yaw) + relative_pos[i][1] * std::cos(ego_yaw);
    }

    double obs_q0 = obs_path.poses[i].pose.orientation.w;
    double obs_q1 = obs_path.poses[i].pose.orientation.x;
    double obs_q2 = obs_path.poses[i].pose.orientation.y;
    double obs_q3 = obs_path.poses[i].pose.orientation.z;
    double obs_yaw = atan2(2 * obs_q1 * obs_q2 + 2 * obs_q0 * obs_q3, -2 * obs_q2 * obs_q2 - 2 * obs_q3 * obs_q3 + 1);
    // 四个顶点
    std::vector<std::vector<double>> obs_rect(4,
      std::vector<double>{obs_path.poses[i].pose.position.x, obs_path.poses[i].pose.position.y});
    for (int i = 0; i < 4; i++) {
      obs_rect[i][0] += relative_pos[i][0] * std::cos(obs_yaw) - relative_pos[i][1] * std::sin(obs_yaw);
      obs_rect[i][1] += relative_pos[i][0] * std::sin(obs_yaw) + relative_pos[i][1] * std::cos(obs_yaw);
    }
    // 不旋转
    // std::vector<std::vector<double>> ego_rect(4,
    //   std::vector<double>{ego_path.poses[i].pose.position.x, ego_path.poses[i].pose.position.y});
    // std::vector<std::vector<double>> obs_rect(4,
    //   std::vector<double>{obs_path.poses[i].pose.position.x, obs_path.poses[i].pose.position.y});
    // for (int i = 0; i < 4; i++) {
    //   ego_rect[i][0] += relative_pos[i][0];
    //   ego_rect[i][1] += relative_pos[i][1];
    //   obs_rect[i][0] += relative_pos[i][0];
    //   obs_rect[i][1] += relative_pos[i][1];
    // }

    if (RectOverlap(ego_rect, obs_rect)) {
      // std::cout << i << "," << ego_path.poses[i].pose.position.x << "," << ego_path.poses[i].pose.position.y << ","
      // << obs_path.poses[i].pose.position.x << "," << obs_path.poses[i].pose.position.y << std::endl;
      return i;
    }
  }
  return -1;
}

bool Mpcc::RectOverlap(std::vector<std::vector<double>>& rec1,
                       std::vector<std::vector<double>>& rec2) {
  for (int i = 0; i < 4; i++) {
    if (InQuad(rec1, rec2[i][0], rec2[i][1]) || InQuad(rec2, rec1[i][0], rec1[i][1])) {
      return true;
    }
  }
  return false;
}

bool Mpcc::CalcuDMPC(const nav_msgs::Path& ego_path, const nav_msgs::Path& obs_path,
                     Eigen::SparseMatrix<double> state,
                     Eigen::SparseMatrix<double>& Cu, Eigen::SparseMatrix<double>& ulow,
                     Eigen::SparseMatrix<double>& uup) {
  int collision_index = CheckCollision(ego_path, obs_path);
  
  // std::cout << collision_index << std::endl;
  // if (collision_index == -1) {
  //   return false;
  // }
  if (collision_index >= 0) {
    double min_r = uav_size + buffer;
    double delta_x = ego_path.poses[collision_index].pose.position.x -
      obs_path.poses[collision_index].pose.position.x;
    double delta_y = ego_path.poses[collision_index].pose.position.y -
      obs_path.poses[collision_index].pose.position.y;
    double xi = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    double rho = min_r * min_r - xi * xi + 2 * delta_x * ego_path.poses[collision_index].pose.position.x +
      2 * delta_y * ego_path.poses[collision_index].pose.position.y;
    Eigen::SparseMatrix<double> mu(1, horizon * state_dim_);
    double epsilon = 0.01;
    // if (collision_index > 0) {
    //   collision_index--;
    // }
    mu.coeffRef(0, collision_index * state_dim_ + 0) = 2 * delta_x;
    mu.coeffRef(0, collision_index * state_dim_ + 2) = 2 * delta_y;

    Cu = mu * BB;
    Eigen::SparseMatrix<double> temp(1, 1);
    temp.coeffRef(0, 0) = rho;
    ulow = temp - mu * AA * state;
  }
  
  uup.resize(1, 1);
  uup.coeffRef(0, 0) = 10000000.0;
  if (collision_index == -1) {
    ulow.coeffRef(0, 0) = -10000000.0;
  }
  return true;
  // if D(iagent,i) == 1
  //   v_ij = P(3*kci(iagent , i)-2:3*kci(iagent , i),iagent)-P(3*kci(iagent , i)-2:3*kci(iagent , i),i);
  //   xi_ij = norm( v_ij);
    
  //   mu_ij = [zeros(3*(kci(iagent , i)-1),1)'  v_ij' zeros(3*(K-kci(iagent , i)),1)'  ]';
  //   rho = r_min*xi_ij - xi_ij^2 + v_ij'*P(3*kci(iagent , i)-2:3*kci(iagent , i),iagent);
    
  //   A = mu_ij'*lambda;
  //   b = rho - mu_ij'*A0*X(:,iagent) + epsilon* xi_ij;

  //   Aieq = [Aieq ; -A; ];
  //   bieq = [bieq ; -b; ];
  // end
}