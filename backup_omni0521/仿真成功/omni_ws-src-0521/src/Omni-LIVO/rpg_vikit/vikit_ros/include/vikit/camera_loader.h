/*
 * camera_loader.h
 * Fixed for Omni-LIVO: Complete Multi-Camera & Mei Model Support
 */

#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <vector>
#include <ros/ros.h>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/polynomial_camera.h>
#include <vikit/params_helper.h>

namespace vk {
namespace camera_loader {

// =========================================================================
// 1. 单相机加载器 (解决调用冲突)
// =========================================================================
bool loadFromRosNs(const std::string& ns, vk::AbstractCamera*& cam)
{
  bool res = true;
  // 确保路径以 / 开头，强制使用绝对路径
  std::string abs_ns = (ns[0] == '/') ? ns : ("/" + ns);
  
  std::string cam_model;
  if(!ros::param::get(abs_ns + "/cam_model", cam_model)) 
  {
      ROS_WARN("Camera Loader: Could not find cam_model at %s", abs_ns.c_str());
      return false;
  }

  // --- 新增：Omni (Mei) 模型支持 ---
  if(cam_model == "omni")
  {
    int w, h;
    double xi, fu, fv, pu, pv;
    bool ok = true;
    ok &= ros::param::get(abs_ns + "/cam_width", w);
    ok &= ros::param::get(abs_ns + "/cam_height", h);
    ok &= ros::param::get(abs_ns + "/cam_xi", xi);
    ok &= ros::param::get(abs_ns + "/cam_fu", fu);
    ok &= ros::param::get(abs_ns + "/cam_fv", fv);
    ok &= ros::param::get(abs_ns + "/cam_pu", pu);
    ok &= ros::param::get(abs_ns + "/cam_pv", pv);

    if(!ok || fu < 1e-6) {
        ROS_ERROR("Omni Model: Missing params or fx=0 at %s", abs_ns.c_str());
        return false;
    }

    double d0=0, d1=0, d2=0, d3=0;
    ros::param::get(abs_ns + "/cam_d0", d0);
    ros::param::get(abs_ns + "/cam_d1", d1);
    ros::param::get(abs_ns + "/cam_d2", d2);
    ros::param::get(abs_ns + "/cam_d3", d3);

    cam = new vk::OmniCamera(w, h, xi, fu, fv, pu, pv, d0, d1, d2, d3);
    ROS_INFO("Successfully loaded Omni camera: %s", abs_ns.c_str());
  }
  // --- 原有：Pinhole 模型 ---
  else if(cam_model == "Pinhole")
  {
    cam = new vk::PinholeCamera(
        getParam<int>(abs_ns+"/cam_width"),
        getParam<int>(abs_ns+"/cam_height"),
        getParam<double>(abs_ns+"/scale", 1.0),
        getParam<double>(abs_ns+"/cam_fx"),
        getParam<double>(abs_ns+"/cam_fy"),
        getParam<double>(abs_ns+"/cam_cx"),
        getParam<double>(abs_ns+"/cam_cy"),
        getParam<double>(abs_ns+"/cam_d0", 0.0),
        getParam<double>(abs_ns+"/cam_d1", 0.0),
        getParam<double>(abs_ns+"/cam_d2", 0.0),
        getParam<double>(abs_ns+"/cam_d3", 0.0));
  }
  // --- 其他模型省略 (保持你原有代码逻辑) ---
  else if(cam_model == "Ocam")
    cam = new vk::OmniCamera(getParam<std::string>(abs_ns+"/cam_calib_file", ""));
  else if(cam_model == "ATAN")
    cam = new vk::ATANCamera(getParam<int>(abs_ns+"/cam_width"), getParam<int>(abs_ns+"/cam_height"),
                             getParam<double>(abs_ns+"/cam_fx"), getParam<double>(abs_ns+"/cam_fy"),
                             getParam<double>(abs_ns+"/cam_cx"), getParam<double>(abs_ns+"/cam_cy"),
                             getParam<double>(abs_ns+"/cam_d0"));
  else
  {
    cam = NULL;
    res = false;
    ROS_ERROR("Unknown camera model: %s", cam_model.c_str());
  }
  return res;
}

// =========================================================================
// 2. 多相机加载器 (通常供 LIVMapper 等调用)
// =========================================================================
bool loadFromRosNs(const std::string& ns, std::vector<vk::AbstractCamera*>& cam_list)
{
  bool all_res = true;
  int cam_num;
  std::string abs_parent_ns = (ns[0] == '/') ? ns : ("/" + ns);

  if(!ros::param::get(abs_parent_ns + "/cam_num", cam_num)) 
  {
      ROS_ERROR("Multi-Cam: Could not find cam_num at %s", abs_parent_ns.c_str());
      return false;
  }
  
  for (int i = 0; i < cam_num; i ++)
  {
    std::string cam_ns = abs_parent_ns + "/cam_" + std::to_string(i);
    vk::AbstractCamera* cam = NULL;
    
    // 调用上面修复过的单相机加载函数
    if(!loadFromRosNs(cam_ns, cam))
    {
      ROS_ERROR("Failed to load camera %d at %s", i, cam_ns.c_str());
      all_res = false;
    }
    else
    {
      cam_list.push_back(cam);
    }
  }
  return all_res;
}

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_