// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#include "mpcc.h"
#include "config.h"

// g11 g12 g21 g22 gx gy的顺序
void Mpcc::chance_constrains_set(std::vector<double>& coeff, std::vector<double> obst,
                                 std::vector<double> ego, const Config& config) {
  double kesi = -1.0;
  double alpha = 0.99;
  double z_alpha = normsinv(1 - alpha / horizon);
  double S_x = 0.4;
  double S_y = 0.1;
  double r_box_x = 0.1;
  double r_box_y = 0.5;
  
  // chance constrains 等式左右除以2，统一kesi为-1.0
  double denom_x = 2 * std::pow(r_box_x + z_alpha * S_x, 2);
  double denom_y = 2 * std::pow(r_box_y + z_alpha * S_y, 2);
  // 不用chance cobstrains, 将denom_x denom_y设为椭圆半径平方
  denom_x = config.obs_x_r * config.obs_x_r;
  denom_y = config.obs_y_r * config.obs_y_r;

  double x0 = obst[0];
  double y0 = obst[1];
  double denom = std::pow(ego[0] - x0, 2) / denom_x +
                 std::pow(ego[1] - y0, 2) / denom_y + kesi;
  double g11 =
      2 *
      (-std::pow(ego[0] - x0, 2) / std::pow(denom_x, 2) +
       std::pow(ego[1] - y0, 2) / (denom_x * denom_y) + kesi / denom_x) /
      std::pow(denom, 2);
  double g12 = -4 * (ego[0] - x0) * (ego[1] - y0) / (denom_x * denom_y) /
               std::pow(denom, 2);
  double g21 = g12;
  double g22 =
      2 *
      (std::pow(ego[0] - x0, 2) / (denom_x * denom_y) -
       std::pow(ego[1] - y0, 2) / std::pow(denom_y, 2) + kesi / denom_y) /
      std::pow(denom, 2);
  double gx = 2 * (ego[0] - x0) / denom_x / denom;
  double gy = 2 * (ego[1] - y0) / denom_y / denom;

  coeff[0] = g11;
  coeff[1] = g12;
  coeff[2] = g21;
  coeff[3] = g22;
  coeff[4] = gx;
  coeff[5] = gy;
}

// 返回标准正态分布累积函数的逆函数。该分布的平均值为 0，标准偏差为 1
double Mpcc::normsinv(const double p) {
  static const double LOW = 0.02425;
  static const double HIGH = 0.97575;

  /* Coefficients in rational approximations. */
  static const double a[] = {-3.969683028665376e+01, 2.209460984245205e+02,
                             -2.759285104469687e+02, 1.383577518672690e+02,
                             -3.066479806614716e+01, 2.506628277459239e+00};

  static const double b[] = {-5.447609879822406e+01, 1.615858368580409e+02,
                             -1.556989798598866e+02, 6.680131188771972e+01,
                             -1.328068155288572e+01};

  static const double c[] = {-7.784894002430293e-03, -3.223964580411365e-01,
                             -2.400758277161838e+00, -2.549732539343734e+00,
                             4.374664141464968e+00,  2.938163982698783e+00};

  static const double d[] = {7.784695709041462e-03, 3.224671290700398e-01,
                             2.445134137142996e+00, 3.754408661907416e+00};
  double q, r;
  errno = 0;

  if (p < 0 || p > 1) {
    errno = EDOM;
    return 0.0;
  } else if (p == 0) {
    errno = ERANGE;
    return -HUGE_VAL /* minus "infinity" */;
  } else if (p == 1) {
    errno = ERANGE;
    return HUGE_VAL /* "infinity" */;
  } else if (p < LOW) {
    /* Rational approximation for lower region */
    q = sqrt(-2 * log(p));
    return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q +
            c[5]) /
           ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
  } else if (p > HIGH) {
    /* Rational approximation for upper region */
    q = sqrt(-2 * log(1 - p));
    return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q +
             c[5]) /
           ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
  } else {
    /* Rational approximation for central region */
    q = p - 0.5;
    r = q * q;
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r +
            a[5]) *
           q /
           (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1);
  }
}
