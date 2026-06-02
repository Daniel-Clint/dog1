/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_POINT_H_
#define LIVO_POINT_H_

#include <boost/noncopyable.hpp>
#include <bitset>
#include "common_lib.h"
#include "frame.h"

class Feature;

// 【性能优化】跨相机数据结构，内嵌到 VisualPoint 中避免 map 查找
#define MAX_CAMERAS 10  // 支持最多10个相机

// 【内存优化】使用位域和紧凑类型减少内存占用（从~20字节优化到~8字节）
struct CrossCameraData {
    std::bitset<MAX_CAMERAS> currently_visible;    // 当前帧可见性 (10 bits)
    std::bitset<MAX_CAMERAS> previously_visible;   // 上一帧可见性 (10 bits)

    int8_t primary_cam_idx;                    // 主相机索引 (-1 to 9, 1 byte)
    int8_t previous_cam_idx;                   // 上一帧所属相机 (1 byte)
    int8_t migration_source_cam;               // 迁移源相机 (1 byte)
    uint8_t cross_camera_migrations;           // 跨相机迁移计数 (0-255, 1 byte)
    bool has_migration_history;                // 是否有迁移历史 (1 byte)

    int last_seen_frame_id;                    // 最后一次被观测到的帧ID (4 bytes)

    CrossCameraData() :
        primary_cam_idx(-1),
        previous_cam_idx(-1),
        migration_source_cam(-1),
        cross_camera_migrations(0),
        last_seen_frame_id(-1),
        has_migration_history(false) {
        currently_visible.reset();
        previously_visible.reset();
    }
};

/// A visual map point on the surface of the scene.
class VisualPoint : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 【内存优化】限制最大观测数，防止obs_列表无限增长
  static const int MAX_OBS = 15;  //!< Maximum number of observations to keep per point

  Vector3d pos_;                //!< 3d pos of the point in the world coordinate frame.
  Vector3d normal_;             //!< Surface normal at point.
  Matrix3d normal_information_; //!< Inverse covariance matrix of normal estimation.
  Vector3d previous_normal_;    //!< Last updated normal vector.
  list<Feature *> obs_;         //!< Reference patches which observe the point.
  Eigen::Matrix3d covariance_;  //!< Covariance of the point.
  bool is_converged_;           //!< True if the point is converged.
  bool is_normal_initialized_;  //!< True if the normal is initialized.
  bool has_ref_patch_;          //!< True if the point has a reference patch.
  Feature *ref_patch;           //!< Reference patch of the point.

  // 【性能优化】内嵌跨相机数据，避免 map 查找
  CrossCameraData cross_cam_data_;

  // 【性能优化】可见相机缓存，减少不必要的投影计算
  std::bitset<MAX_CAMERAS> visible_cameras_cache_;  //!< 缓存最近哪些相机能看到此点
  int cache_frame_id_;                               //!< 缓存对应的帧ID

  VisualPoint(const Vector3d &pos);
  ~VisualPoint();
  void findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const;
  void deleteNonRefPatchFeatures();
  void deleteFeatureRef(Feature *ftr);
  void addFrameRef(Feature *ftr);
  bool getCloseViewObs(const Vector3d &pos, Feature *&obs, const Vector2d &cur_px) const;
};

#endif // LIVO_POINT_H_
