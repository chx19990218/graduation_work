// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include "cubic_spline.h"
// #include "types.h"
// #include "Params/params.h"
#include "config.h"
#include <iostream>
#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>

//return value
struct RawPath{
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
};
// data struct
struct PathData{
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    Eigen::VectorXd s;
    int n_points;
};

class ArcLengthSpline {
public:
    // X and Y spline used for final spline fit
    void gen2DSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y);
    Eigen::Vector2d getPostion(double) const;
    Eigen::Vector2d getDerivative(double) const;
    Eigen::Vector2d getSecondDerivative(double) const;
    double getLength() const;
    double porjectOnSpline(double x, double y) const;

    ArcLengthSpline();

    void setData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in);
    void setRegularData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in,const Eigen::VectorXd &s_in);
    Eigen::VectorXd compArcLength(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in) const;
    PathData resamplePath(const CubicSpline &initial_spline_x,const CubicSpline &initial_spline_y,double total_arc_length) const;
    RawPath outlierRemoval(const Eigen::VectorXd &X_original,const Eigen::VectorXd &Y_original) const;
    void fitSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y);
    double unwrapInput(double x) const;

    PathData path_data_;      // initial data and data used for successive fitting
//    PathData pathDataFinal; // final data
    CubicSpline spline_x_;
    CubicSpline spline_y_;
};