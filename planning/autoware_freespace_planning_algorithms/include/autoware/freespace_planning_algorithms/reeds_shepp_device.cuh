// Copyright 2026. Generated device port of Reeds-Shepp distance for CUDA Hybrid A*.
// Based on autoware_freespace_planning_algorithms/src/reeds_shepp.cpp (BSD-3-Clause upstream).
#ifndef AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__REEDS_SHEPP_DEVICE_CUH_
#define AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__REEDS_SHEPP_DEVICE_CUH_

#include <cmath>
#include <limits>

#ifdef __CUDACC__
#define RS_HD __host__ __device__ __forceinline__
#else
#define RS_HD inline
#endif

namespace autoware::freespace_planning_algorithms::reeds_shepp_device
{

struct StateXYT
{
  double x{};
  double y{};
  double yaw{};
};

struct DevicePath
{
  double length_[5]{};
  double totalLength_{std::numeric_limits<double>::max()};

  RS_HD DevicePath() {}
  RS_HD DevicePath(double t, double u = 0., double v = 0., double w = 0., double x = 0.)
  {
    length_[0] = t;
    length_[1] = u;
    length_[2] = v;
    length_[3] = w;
    length_[4] = x;
    totalLength_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
  }
  RS_HD double length() const { return totalLength_; }
};


// The comments, variable names, etc. use the nomenclature from the Reeds & Shepp paper.

const double pi = M_PI;
const double twopi = 2. * pi;
const double RS_EPS = 1e-6;  // used only in assertions
const double ZERO = 10 * std::numeric_limits<double>::epsilon();

RS_HD double mod2pi(double x)
{
  double v = fmod(x, twopi);
  if (v < -pi) {
    v += twopi;
  } else if (v > pi) {
    v -= twopi;
  }
  return v;
}
RS_HD void polar(double x, double y, double & r, double & theta)
{
  r = sqrt(x * x + y * y);
  theta = atan2(y, x);
}
RS_HD void tauOmega(
  double u, double v, double xi, double eta, double phi, double & tau, double & omega)
{
  double delta = mod2pi(u - v);
  double A = sin(u) - sin(delta);
  double B = cos(u) - cos(delta) - 1.;
  double t1 = atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
  tau = (t2 < 0) ? mod2pi(t1 + pi) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}

// formula 8.1 in Reeds-Shepp paper
RS_HD bool LpSpLp(double x, double y, double phi, double & t, double & u, double & v)
{
  polar(x - sin(phi), y - 1. + cos(phi), u, t);
  if (t >= -ZERO) {
    v = mod2pi(phi - t);
    if (v >= -ZERO) {
      return true;
    }
  }
  return false;
}
// formula 8.2
RS_HD bool LpSpRp(double x, double y, double phi, double & t, double & u, double & v)
{
  double t1, u1;
  polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
  u1 = u1 * u1;
  if (u1 >= 4.) {
    double theta;
    u = sqrt(u1 - 4.);
    theta = atan2(2., u);
    t = mod2pi(t1 + theta);
    v = mod2pi(t - phi);
    return t >= -ZERO && v >= -ZERO;
  }
  return false;
}
RS_HD void CSC(double x, double y, double phi, DevicePath & path)
{
  double t, u, v, L_min = path.length(), L;
  if (LpSpLp(x, y, phi, t, u, v) && L_min > (L = abs(t) + abs(u) + abs(v))) {
    path =
      DevicePath(t, u, v);
    L_min = L;
  }
  if (
    LpSpLp(-x, y, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip
  {
    path = DevicePath(-t, -u, -v);
    L_min = L;
  }
  if (
    LpSpLp(x, -y, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // reflect
  {
    path =
      DevicePath(t, u, v);
    L_min = L;
  }
  if (
    LpSpLp(-x, -y, phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip + reflect
  {
    path = DevicePath(-t, -u, -v);
    L_min = L;
  }
  if (LpSpRp(x, y, phi, t, u, v) && L_min > (L = abs(t) + abs(u) + abs(v))) {
    path =
      DevicePath(t, u, v);
    L_min = L;
  }
  if (
    LpSpRp(-x, y, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip
  {
    path = DevicePath(-t, -u, -v);
    L_min = L;
  }
  if (
    LpSpRp(x, -y, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // reflect
  {
    path =
      DevicePath(t, u, v);
    L_min = L;
  }
  // time flip + reflect
  if (LpSpRp(-x, -y, phi, t, u, v) && L_min > (abs(t) + abs(u) + abs(v))) {
    path = DevicePath(-t, -u, -v);
  }
}
// formula 8.3 / 8.4  *** TYPO IN PAPER ***
RS_HD bool LpRmL(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
  polar(xi, eta, u1, theta);
  if (u1 <= 4.) {
    u = -2. * asin(.25 * u1);
    t = mod2pi(theta + .5 * u + pi);
    v = mod2pi(phi - t + u);
    return t >= -ZERO && u <= ZERO;
  }
  return false;
}
RS_HD void CCC(double x, double y, double phi, DevicePath & path)
{
  double t, u, v, L_min = path.length(), L;
  if (LpRmL(x, y, phi, t, u, v) && L_min > (L = abs(t) + abs(u) + abs(v))) {
    path =
      DevicePath(t, u, v);
    L_min = L;
  }
  if (LpRmL(-x, y, -phi, t, u, v) && L_min > (L = abs(t) + abs(u) + abs(v))) {
    // time flip
    path =
      DevicePath(-t, -u, -v);
    L_min = L;
  }
  if (
    LpRmL(x, -y, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // reflect
  {
    path =
      DevicePath(t, u, v);
    L_min = L;
  }
  if (
    LpRmL(-x, -y, phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip + reflect
  {
    path =
      DevicePath(-t, -u, -v);
    L_min = L;
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
  if (LpRmL(xb, yb, phi, t, u, v) && L_min > (L = abs(t) + abs(u) + abs(v))) {
    path =
      DevicePath(v, u, t);
    L_min = L;
  }
  if (
    LpRmL(-xb, yb, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip
  {
    path =
      DevicePath(-v, -u, -t);
    L_min = L;
  }
  if (
    LpRmL(xb, -yb, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // reflect
  {
    path =
      DevicePath(v, u, t);
    L_min = L;
  }
  if (
    LpRmL(-xb, -yb, phi, t, u, v) &&
    L_min > (abs(t) + abs(u) + abs(v)))  // time flip + reflect
  {
    path =
      DevicePath(-v, -u, -t);
  }
}
// formula 8.7
RS_HD bool LpRupLumRm(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi),
         rho = .25 * (2. + sqrt(xi * xi + eta * eta));
  if (rho <= 1.) {
    u = acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    return t >= -ZERO && v <= ZERO;
  }
  return false;
}
// formula 8.8
RS_HD bool LpRumLumRp(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi),
         rho = (20. - xi * xi - eta * eta) / 16.;
  if (rho >= 0 && rho <= 1) {
    u = -acos(rho);
    if (u >= -.5 * pi) {
      tauOmega(u, u, xi, eta, phi, t, v);
      return t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}
RS_HD void CCCC(double x, double y, double phi, DevicePath & path)
{
  double t, u, v, L_min = path.length(), L;
  if (
    LpRupLumRm(x, y, phi, t, u, v) && L_min > (L = abs(t) + 2. * abs(u) + abs(v))) {
    path = DevicePath(t, u, -u, v);
    L_min = L;
  }
  if (
    LpRupLumRm(-x, y, -phi, t, u, v) &&
    L_min > (L = abs(t) + 2. * abs(u) + abs(v)))  // time flip
  {
    path = DevicePath(-t, -u, u, -v);
    L_min = L;
  }
  if (
    LpRupLumRm(x, -y, -phi, t, u, v) &&
    L_min > (L = abs(t) + 2. * abs(u) + abs(v)))  // reflect
  {
    path = DevicePath(t, u, -u, v);
    L_min = L;
  }
  if (
    LpRupLumRm(-x, -y, phi, t, u, v) &&
    L_min > (L = abs(t) + 2. * abs(u) + abs(v)))  // time flip + reflect
  {
    path = DevicePath(-t, -u, u, -v);
    L_min = L;
  }

  if (
    LpRumLumRp(x, y, phi, t, u, v) && L_min > (L = abs(t) + 2. * abs(u) + abs(v))) {
    path =
      DevicePath(t, u, u, v);
    L_min = L;
  }
  if (
    LpRumLumRp(-x, y, -phi, t, u, v) &&
    L_min > (L = abs(t) + 2. * abs(u) + abs(v)))  // time flip
  {
    path = DevicePath(-t, -u, -u, -v);
    L_min = L;
  }
  if (
    LpRumLumRp(x, -y, -phi, t, u, v) &&
    L_min > (L = abs(t) + 2. * abs(u) + abs(v)))  // reflect
  {
    path =
      DevicePath(t, u, u, v);
    L_min = L;
  }
  if (
    LpRumLumRp(-x, -y, phi, t, u, v) &&
    L_min > (abs(t) + 2. * abs(u) + abs(v)))  // time flip + reflect
  {
    path = DevicePath(-t, -u, -u, -v);
  }
}
// formula 8.9
RS_HD bool LpRmSmLm(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.) {
    double r = sqrt(rho * rho - 4.);
    u = 2. - r;
    t = mod2pi(theta + atan2(r, -2.));
    v = mod2pi(phi - .5 * pi - t);
    return t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}
// formula 8.10
RS_HD bool LpRmSmRm(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
  polar(-eta, xi, rho, theta);
  if (rho >= 2.) {
    t = theta;
    u = 2. - rho;
    v = mod2pi(t + .5 * pi - phi);
    return t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}
RS_HD void CCSC(double x, double y, double phi, DevicePath & path)
{
  double t, u, v, L_min = path.length() - .5 * pi, L;
  if (LpRmSmLm(x, y, phi, t, u, v) && L_min > (L = abs(t) + abs(u) + abs(v))) {
    path = DevicePath(t, -.5 * pi, u, v);
    L_min = L;
  }
  if (
    LpRmSmLm(-x, y, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip
  {
    path = DevicePath(-t, .5 * pi, -u, -v);
    L_min = L;
  }
  if (
    LpRmSmLm(x, -y, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // reflect
  {
    path = DevicePath(t, -.5 * pi, u, v);
    L_min = L;
  }
  if (
    LpRmSmLm(-x, -y, phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip + reflect
  {
    path = DevicePath(-t, .5 * pi, -u, -v);
    L_min = L;
  }

  if (LpRmSmRm(x, y, phi, t, u, v) && L_min > (L = abs(t) + abs(u) + abs(v))) {
    path = DevicePath(t, -.5 * pi, u, v);
    L_min = L;
  }
  if (
    LpRmSmRm(-x, y, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip
  {
    path = DevicePath(-t, .5 * pi, -u, -v);
    L_min = L;
  }
  if (
    LpRmSmRm(x, -y, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // reflect
  {
    path = DevicePath(t, -.5 * pi, u, v);
    L_min = L;
  }
  if (
    LpRmSmRm(-x, -y, phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip + reflect
  {
    path = DevicePath(-t, .5 * pi, -u, -v);
    L_min = L;
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
  if (LpRmSmLm(xb, yb, phi, t, u, v) && L_min > (L = abs(t) + abs(u) + abs(v))) {
    path = DevicePath(v, u, -.5 * pi, t);
    L_min = L;
  }
  if (
    LpRmSmLm(-xb, yb, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip
  {
    path = DevicePath(-v, -u, .5 * pi, -t);
    L_min = L;
  }
  if (
    LpRmSmLm(xb, -yb, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // reflect
  {
    path = DevicePath(v, u, -.5 * pi, t);
    L_min = L;
  }
  if (
    LpRmSmLm(-xb, -yb, phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip + reflect
  {
    path = DevicePath(-v, -u, .5 * pi, -t);
    L_min = L;
  }

  if (LpRmSmRm(xb, yb, phi, t, u, v) && L_min > (L = abs(t) + abs(u) + abs(v))) {
    path = DevicePath(v, u, -.5 * pi, t);
    L_min = L;
  }
  if (
    LpRmSmRm(-xb, yb, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip
  {
    path = DevicePath(-v, -u, .5 * pi, -t);
    L_min = L;
  }
  if (
    LpRmSmRm(xb, -yb, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // reflect
  {
    path = DevicePath(v, u, -.5 * pi, t);
    L_min = L;
  }
  if (
    LpRmSmRm(-xb, -yb, phi, t, u, v) &&
    L_min > (abs(t) + abs(u) + abs(v)))  // time flip + reflect
  {
    path = DevicePath(-v, -u, .5 * pi, -t);
  }
}
// formula 8.11 *** TYPO IN PAPER ***
RS_HD bool LpRmSLmRp(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.) {
    u = 4. - sqrt(rho * rho - 4.);
    if (u <= ZERO) {
      t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
      v = mod2pi(t - phi);
      return t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}
RS_HD void CCSCC(double x, double y, double phi, DevicePath & path)
{
  double t, u, v, L_min = path.length() - pi, L;
  if (LpRmSLmRp(x, y, phi, t, u, v) && L_min > (L = abs(t) + abs(u) + abs(v))) {
    path = DevicePath(t, -.5 * pi, u, -.5 * pi, v);
    L_min = L;
  }
  if (
    LpRmSLmRp(-x, y, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // time flip
  {
    path = DevicePath(-t, .5 * pi, -u, .5 * pi, -v);
    L_min = L;
  }
  if (
    LpRmSLmRp(x, -y, -phi, t, u, v) &&
    L_min > (L = abs(t) + abs(u) + abs(v)))  // reflect
  {
    path = DevicePath(t, -.5 * pi, u, -.5 * pi, v);
    L_min = L;
  }
  if (
    LpRmSLmRp(-x, -y, phi, t, u, v) &&
    L_min > (abs(t) + abs(u) + abs(v)))  // time flip + reflect
  {
    path = DevicePath(-t, .5 * pi, -u, .5 * pi, -v);
  }
}

RS_HD DevicePath reedsShepp(double x, double y, double phi)
{
  DevicePath path;
  CSC(x, y, phi, path);
  CCC(x, y, phi, path);
  CCCC(x, y, phi, path);
  CCSC(x, y, phi, path);
  CCSCC(x, y, phi, path);
  return path;
}

RS_HD double distance(const StateXYT & s0, const StateXYT & s1, double rho)
{
  const double dx = s1.x - s0.x;
  const double dy = s1.y - s0.y;
  const double dth = s1.yaw - s0.yaw;
  const double c = cos(s0.yaw);
  const double s = sin(s0.yaw);
  const double x = (c * dx + s * dy) / rho;
  const double y = (-s * dx + c * dy) / rho;
  return rho * reedsShepp(x, y, dth).length();
}

}  // namespace autoware::freespace_planning_algorithms::reeds_shepp_device

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__REEDS_SHEPP_DEVICE_CUH_
