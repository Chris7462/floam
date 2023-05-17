// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

// local header
#include "floam/lidar_optimization.hpp"


void FloamVertex::setToOriginImpl()
{
  _estimate = Sophus::SE3d();
}

void FloamVertex::oplusImpl(const double* update)
{
  Eigen::Matrix<double, 6, 1> delta_r;
  delta_r << update[0], update[1], update[2], update[3], update[4], update[5];

  _estimate = Sophus::SE3d::exp(delta_r) * _estimate; // left multiplication
  // _estimate = _estimate * Sophus::SE3d::exp(delta_r); // right multiplication
}

bool FloamVertex::read(std::istream&)
{
  return false;
}

bool FloamVertex::write(std::ostream&) const
{
  return false;
}

FloamEdge::FloamEdge(Eigen::Vector3d pa, Eigen::Vector3d pb, Eigen::Vector3d cp)
  : BaseUnaryEdge(), lp_a(pa), lp_b(pb), c_p(cp)
{
}

void FloamEdge::computeError()
{
  const FloamVertex* v = static_cast<const FloamVertex*>(_vertices[0]);
  const Sophus::SE3d T = v->estimate();
  Eigen::Vector3d lp = T * c_p;
  Eigen::Vector3d nu = (lp - lp_a).cross(lp - lp_b);
  Eigen::Vector3d de = lp_a - lp_b;
  double de_norm = de.norm();
  double nu_norm = nu.norm();

  _error(0, 0) = nu_norm / de_norm;
}

void FloamEdge::linearizeOplus()
{
  const FloamVertex* v = static_cast<const FloamVertex*>(_vertices[0]);
  const Sophus::SE3d T = v->estimate();
  Eigen::Matrix3d skew_lp = Sophus::SO3d::hat(T * c_p);
  Eigen::Vector3d lp = T * c_p;
  Eigen::Vector3d nu = (lp - lp_a).cross(lp - lp_b);
  Eigen::Vector3d de = lp_a - lp_b;
  double de_norm = de.norm();
  Eigen::Matrix<double, 3, 6> dp_by_se3;
  (dp_by_se3.block<3, 3>(0, 0)).setIdentity();
  dp_by_se3.block<3, 3>(0, 3) = -skew_lp;
  Eigen::Matrix3d skew_de = Sophus::SO3d::hat(lp_a - lp_b);
  _jacobianOplusXi.block<1, 6>(0, 0) = -nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
}

bool FloamEdge::read(std::istream&)
{
  return false;
}

bool FloamEdge::write(std::ostream&) const
{
  return false;
}


FloamSurf::FloamSurf(Eigen::Vector3d cur_p, Eigen::Vector3d p_nor)
  : BaseUnaryEdge(), c_p(cur_p), p_n(p_nor)
{
}

void FloamSurf::computeError()
{
  const FloamVertex *v = static_cast<const FloamVertex *>(_vertices[0]);
  const Sophus::SE3d T = v->estimate();
  Eigen::Vector3d point_w = T * c_p;

  _error(0, 0) = p_n.dot(point_w) + _measurement;
}

void FloamSurf::linearizeOplus()
{
  const FloamVertex* v = static_cast<const FloamVertex*>(_vertices[0]);
  const Sophus::SE3d T = v->estimate();
  Eigen::Matrix3d skew_point_w = Sophus::SO3d::hat(T * c_p);
  Eigen::Matrix<double, 3, 6> dp_by_se3;
  (dp_by_se3.block<3, 3>(0, 0)).setIdentity();
  dp_by_se3.block<3, 3>(0, 3) = -skew_point_w;
  _jacobianOplusXi.block<1, 6>(0, 0) = p_n.transpose() * dp_by_se3;
}

bool FloamSurf::read(std::istream&)
{
  return false;
}

bool FloamSurf::write(std::ostream&) const
{
  return false;
}
