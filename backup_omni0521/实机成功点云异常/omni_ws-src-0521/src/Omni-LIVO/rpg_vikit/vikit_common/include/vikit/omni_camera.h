/*
 * omni_camera.h
 * Fixed for Omni-LIVO: Mei (Unified) Camera Model Implementation
 */

#ifndef VIKIT_OMNI_CAMERA_H_
#define VIKIT_OMNI_CAMERA_H_

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vikit/abstract_camera.h>

namespace vk {

using namespace std;
using namespace Eigen;

class OmniCamera : public AbstractCamera {

private:
  // Mei 模型特有参数
  double xi_;
  double k1_, k2_;  // 径向畸变
  double p1_, p2_;  // 切向畸变
  double gamma1_, gamma2_; // fu, fv
  double u0_, v0_;  // cu, cv

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double error_multiplier_;

  // -------------------------------------------------------------------------
  // 构造函数
  // -------------------------------------------------------------------------
  
  // 默认构造
  OmniCamera() : AbstractCamera(0,0,1.0), xi_(0), k1_(0), k2_(0), p1_(0), p2_(0), 
                 gamma1_(0), gamma2_(0), u0_(0), v0_(0), error_multiplier_(1.0) {}

  // 占位构造 (防止旧代码调用报错)
  OmniCamera(string calibFile) : AbstractCamera(0,0,1.0) {
      std::cerr << "[OmniCamera] Warning: File loading not implemented for Mei model." << std::endl;
  }

  // [关键] 真正被调用的构造函数
  // 修复：AbstractCamera 构造函数增加第3个参数 scale = 1.0
  OmniCamera(int width, int height, 
             double xi, 
             double fu, double fv, 
             double u0, double v0, 
             double k1, double k2, double p1, double p2)
  : AbstractCamera(width, height, 1.0), 
    xi_(xi), 
    k1_(k1), k2_(k2), 
    p1_(p1), p2_(p2), 
    gamma1_(fu), gamma2_(fv), 
    u0_(u0), v0_(v0)
  {
      error_multiplier_ = fabs(gamma1_); 
  }

  ~OmniCamera() {}

  // -------------------------------------------------------------------------
  // 核心算法：投影 (3D -> 2D)
  // -------------------------------------------------------------------------
  virtual Vector2d world2cam(const Vector3d& xyz_c) const
  {
      Vector2d uv_rect;
      double x = xyz_c[0];
      double y = xyz_c[1];
      double z = xyz_c[2];

      double r = sqrt(x*x + y*y + z*z);
      if (r < 1e-6) return Vector2d(0,0);
      
      double xs = x / r;
      double ys = y / r;
      double zs = z / r;

      // Mei 模型投影
      double denom = zs + xi_;
      if (fabs(denom) < 1e-6) return Vector2d(0,0);

      double xu = xs / denom;
      double yu = ys / denom;

      // 畸变
      double r2 = xu*xu + yu*yu;
      double r4 = r2*r2;
      double rad_dist = 1 + k1_*r2 + k2_*r4;
      
      double xd = xu * rad_dist + 2*p1_*xu*yu + p2_*(r2 + 2*xu*xu);
      double yd = yu * rad_dist + p1_*(r2 + 2*yu*yu) + 2*p2_*xu*yu;

      // 像素坐标
      uv_rect[0] = gamma1_ * xd + u0_;
      uv_rect[1] = gamma2_ * yd + v0_;

      return uv_rect;
  }

  // 兼容接口
  virtual Vector2d world2cam(const Vector2d& uv) const { return Vector2d(0,0); }

  // -------------------------------------------------------------------------
  // 核心算法：反投影 (2D -> 3D)
  // -------------------------------------------------------------------------
  virtual Vector3d cam2world(const double& u, const double& v) const
  {
      double mx_d = (u - u0_) / gamma1_;
      double my_d = (v - v0_) / gamma2_;

      // 去畸变 (迭代)
      double mx_u = mx_d;
      double my_u = my_d;
      for(int i=0; i<8; ++i)
      {
          double r2 = mx_u*mx_u + my_u*my_u;
          double r4 = r2*r2;
          double rad_dist = 1.0 + k1_*r2 + k2_*r4;
          double dx_u = 2.0*p1_*mx_u*my_u + p2_*(r2 + 2.0*mx_u*mx_u);
          double dy_u = p1_*(r2 + 2.0*my_u*my_u) + 2.0*p2_*mx_u*my_u;
          
          mx_u = (mx_d - dx_u) / rad_dist;
          my_u = (my_d - dy_u) / rad_dist;
      }

      // 映射回单位球
      double r2_u = mx_u*mx_u + my_u*my_u;
      double lambda = (xi_ + sqrt(1.0 + (1.0 - xi_*xi_) * r2_u)) / (1.0 + r2_u);
      
      return Vector3d(lambda * mx_u, lambda * my_u, lambda - xi_);
  }

  virtual Vector3d cam2world(const Vector2d& px) const {
      return cam2world(px[0], px[1]);
  }

  // -------------------------------------------------------------------------
  // 辅助函数
  // -------------------------------------------------------------------------
  virtual double errorMultiplier2() const { return fabs(gamma1_); }
  virtual double errorMultiplier() const { return fabs(gamma1_); }
  
  // 防止 AbstractCamera 纯虚函数报错
  virtual double fx() const { return gamma1_; };
  virtual double fy() const { return gamma2_; };
  virtual double cx() const { return u0_; };
  virtual double cy() const { return v0_; };
};

} // namespace vk

#endif // VIKIT_OMNI_CAMERA_H_