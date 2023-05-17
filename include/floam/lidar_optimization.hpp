#pragma once

#include <iostream>

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>


class FLOAMVertex: public g2o::BaseVertex<6, Sophus::SE3d>
{
  public:
    virtual void setToOriginImpl() override;
    virtual void oplusImpl(const double* update) override;
    virtual bool read(std::istream& in);
    virtual bool write(std::ostream& out) const;
};

class FLOAMEdge: public g2o::BaseUnaryEdge<1, double, FLOAMVertex>
{
  public:
    FLOAMEdge(Eigen::Vector3d pa, Eigen::Vector3d pb, Eigen::Vector3d cp);
    virtual void computeError() override;
    virtual void linearizeOplus() override;
    virtual bool read(std::istream& in);
    virtual bool write(std::ostream& out) const;

  private:
    Eigen::Vector3d lp_a, lp_b, c_p;
};

class FLOAMSurf: public g2o::BaseUnaryEdge<1, double, FLOAMVertex>
{
  public:
    FLOAMSurf(Eigen::Vector3d cur_p, Eigen::Vector3d p_nor);
    virtual void computeError() override;
    virtual void linearizeOplus() override;
    virtual bool read(std::istream& in);
    virtual bool write(std::ostream& out) const;

  private:
    Eigen::Vector3d c_p, p_n;
};
