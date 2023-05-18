#pragma once

// system header
#include <iostream>

// g2o header
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>

// eigen
#include <Eigen/Dense>

// sophus
#include <sophus/se3.hpp>


class FloamVertex: public g2o::BaseVertex<6, Sophus::SE3d>
{
  public:
    virtual void setToOriginImpl() override;
    virtual void oplusImpl(const double* update) override;
    virtual bool read(std::istream&);
    virtual bool write(std::ostream&) const;
};

class FloamEdge: public g2o::BaseUnaryEdge<1, double, FloamVertex>
{
  public:
    FloamEdge( Eigen::Vector3d curr_point, Eigen::Vector3d last_point_a, Eigen::Vector3d last_point_b);
    virtual void computeError() override;
    virtual void linearizeOplus() override;
    virtual bool read(std::istream&);
    virtual bool write(std::ostream&) const;

  private:
    Eigen::Vector3d curr_point_, last_point_a_, last_point_b_;
};

class FloamSurf: public g2o::BaseUnaryEdge<1, double, FloamVertex>
{
  public:
    FloamSurf(Eigen::Vector3d curr_point, Eigen::Vector3d plane_unit_norm);
    virtual void computeError() override;
    virtual void linearizeOplus() override;
    virtual bool read(std::istream&);
    virtual bool write(std::ostream&) const;

  private:
    Eigen::Vector3d curr_point_, plane_unit_norm_;
};
