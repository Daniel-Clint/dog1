#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

struct PairHash {
  std::size_t operator()(const std::pair<int, int>& key) const {
    return (static_cast<std::size_t>(static_cast<uint32_t>(key.first)) << 32U) ^
           static_cast<std::size_t>(static_cast<uint32_t>(key.second));
  }
};

class LightTerrainAnalysis {
 public:
  LightTerrainAnalysis() : pnh_("~"), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "/body_cloud");
    pnh_.param<std::string>("body_frame", body_frame_, "base_link");
    pnh_.param<std::string>("world_frame", world_frame_, "map");
    pnh_.param<std::string>("output_map_topic", output_map_topic_, "/map");

    pnh_.param<double>("z_min", z_min_, -0.1);
    pnh_.param<double>("z_max", z_max_, 0.2);
    pnh_.param<double>("max_range", max_range_, 15.0);
    pnh_.param<double>("grid_size", grid_size_, 0.1);
    pnh_.param<double>("area_size", area_size_, 50.0);

    pnh_.param<int>("hit_delta", hit_delta_, 15);
    pnh_.param<int>("free_delta", free_delta_, -5);
    pnh_.param<int>("hit_upper", hit_upper_, 128);
    pnh_.param<int>("hit_lower", hit_lower_, 0);
    if (hit_upper_ <= hit_lower_) {
      ROS_WARN("hit_upper (%d) <= hit_lower (%d), reset to 128/0", hit_upper_, hit_lower_);
      hit_upper_ = 128;
      hit_lower_ = 0;
    }
    if (hit_delta_ < 0) {
      ROS_WARN("hit_delta (%d) < 0, reset to 15", hit_delta_);
      hit_delta_ = 15;
    }
    if (free_delta_ > 0) {
      ROS_WARN("free_delta (%d) > 0, reset to -5", free_delta_);
      free_delta_ = -5;
    }

    pnh_.param<bool>("save_2d_grid", save_2d_grid_, true);
    pnh_.param<std::string>("save_dir", save_dir_, "");

    cloud_sub_ = nh_.subscribe(input_cloud_topic_, 1, &LightTerrainAnalysis::cloudCallback, this);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(output_map_topic_, 1, true);

    const std::time_t t = std::time(nullptr);
    std::tm tm_local{};
    localtime_r(&t, &tm_local);
    std::ostringstream oss;
    oss << (tm_local.tm_year + 1900) << "-"
        << std::setw(2) << std::setfill('0') << (tm_local.tm_mon + 1) << "-"
        << std::setw(2) << std::setfill('0') << tm_local.tm_mday << "-"
        << std::setw(2) << std::setfill('0') << tm_local.tm_hour << "-"
        << std::setw(2) << std::setfill('0') << tm_local.tm_min << "-"
        << std::setw(2) << std::setfill('0') << tm_local.tm_sec;
    save_timestamp_ = oss.str();

    ROS_INFO_STREAM("light_terrain_analysis listening on " << input_cloud_topic_
                    << ", publishing " << output_map_topic_
                    << " | log-odds hit_delta=" << hit_delta_
                    << " free_delta=" << free_delta_
                    << " [" << hit_lower_ << ", " << hit_upper_ << "]");
  }

  void saveMapIfNeeded() {
    if (!save_2d_grid_) {
      return;
    }
    if (!initialized_) {
      ROS_WARN("Skip saving map: no valid map has been initialized.");
      return;
    }
    if (save_dir_.empty()) {
      ROS_WARN("Skip saving map: save_dir is empty.");
      return;
    }

    const std::string command = "mkdir -p \"" + save_dir_ + "\"";
    std::ignore = std::system(command.c_str());

    cv::Mat image(height_, width_, CV_8UC1, cv::Scalar(127));
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const int idx = toFlat(x, y);
        if (!known_flags_[idx]) {
          image.at<uint8_t>(height_ - 1 - y, x) = static_cast<uint8_t>(127);
          continue;
        }
        const int occ = static_cast<int>((static_cast<int>(log_odds_[idx]) * 100) / hit_upper_);
        const int pixel = 255 - (occ * 255 / 100);
        image.at<uint8_t>(height_ - 1 - y, x) = static_cast<uint8_t>(std::max(0, std::min(255, pixel)));
      }
    }

    const std::string file_path = save_dir_ + "/" + save_timestamp_ + ".png";
    if (cv::imwrite(file_path, image)) {
      ROS_INFO_STREAM("Saved 2D grid map to: " << file_path);
    } else {
      ROS_WARN_STREAM("Failed to save 2D grid map to: " << file_path);
    }
  }

 private:
  struct Point2D {
    double x{0.0};
    double y{0.0};
  };

  static Point2D transformPoint2D(const tf2::Transform& tf, double x, double y, double z) {
    tf2::Vector3 p(x, y, z);
    const tf2::Vector3 out = tf * p;
    return Point2D{out.x(), out.y()};
  }

  static tf2::Transform toTf(const geometry_msgs::TransformStamped& t) {
    tf2::Transform tf;
    tf2::fromMsg(t.transform, tf);
    return tf;
  }

  static bool isFinite(const pcl::PointXYZ& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }

  int toFlat(int x, int y) const { return y * width_ + x; }

  std::pair<int, int> worldToCell(double wx, double wy) const {
    const int cx = static_cast<int>(std::floor((wx - origin_x_) / grid_size_));
    const int cy = static_cast<int>(std::floor((wy - origin_y_) / grid_size_));
    return {cx, cy};
  }

  std::pair<double, double> cellToWorldCenter(int cx, int cy) const {
    const double wx = (static_cast<double>(cx) + 0.5) * grid_size_;
    const double wy = (static_cast<double>(cy) + 0.5) * grid_size_;
    return {wx, wy};
  }

  void initializeMap(double cx, double cy) {
    const int side_cells = std::max(1, static_cast<int>(std::ceil(area_size_ / grid_size_)));
    width_ = side_cells;
    height_ = side_cells;
    origin_x_ = cx - (static_cast<double>(width_) * grid_size_ * 0.5);
    origin_y_ = cy - (static_cast<double>(height_) * grid_size_ * 0.5);
    log_odds_.assign(width_ * height_, 0);
    known_flags_.assign(width_ * height_, 0);
    initialized_ = true;
  }

  void ensureInside(double wx, double wy) {
    if (!initialized_) {
      initializeMap(wx, wy);
      return;
    }

    const double cur_min_x = origin_x_;
    const double cur_min_y = origin_y_;
    const double cur_max_x = origin_x_ + static_cast<double>(width_) * grid_size_;
    const double cur_max_y = origin_y_ + static_cast<double>(height_) * grid_size_;

    if (wx >= cur_min_x && wx < cur_max_x && wy >= cur_min_y && wy < cur_max_y) {
      return;
    }

    double new_min_x = cur_min_x;
    double new_min_y = cur_min_y;
    double new_max_x = cur_max_x;
    double new_max_y = cur_max_y;

    const double half_expand = area_size_ * 0.5;
    if (wx < cur_min_x) {
      new_min_x = wx - half_expand;
    } else if (wx >= cur_max_x) {
      new_max_x = wx + half_expand;
    }
    if (wy < cur_min_y) {
      new_min_y = wy - half_expand;
    } else if (wy >= cur_max_y) {
      new_max_y = wy + half_expand;
    }

    const int new_width = std::max(1, static_cast<int>(std::ceil((new_max_x - new_min_x) / grid_size_)));
    const int new_height = std::max(1, static_cast<int>(std::ceil((new_max_y - new_min_y) / grid_size_)));

    std::vector<uint8_t> new_log_odds(new_width * new_height, 0);
    std::vector<uint8_t> new_known_flags(new_width * new_height, 0);

    const int offset_x = static_cast<int>(std::llround((origin_x_ - new_min_x) / grid_size_));
    const int offset_y = static_cast<int>(std::llround((origin_y_ - new_min_y) / grid_size_));

    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const int old_idx = toFlat(x, y);
        const int nx = x + offset_x;
        const int ny = y + offset_y;
        if (nx < 0 || ny < 0 || nx >= new_width || ny >= new_height) {
          continue;
        }
        const int new_idx = ny * new_width + nx;
        new_log_odds[new_idx] = log_odds_[old_idx];
        new_known_flags[new_idx] = known_flags_[old_idx];
      }
    }

    width_ = new_width;
    height_ = new_height;
    origin_x_ = new_min_x;
    origin_y_ = new_min_y;
    log_odds_.swap(new_log_odds);
    known_flags_.swap(new_known_flags);
  }

  void applyDelta(int cx, int cy, int delta) {
    if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) {
      return;
    }
    const int idx = toFlat(cx, cy);
    if (!known_flags_[idx]) {
      known_flags_[idx] = 1;
      log_odds_[idx] = 0;
    }
    int value = static_cast<int>(log_odds_[idx]) + delta;
    value = std::max(hit_lower_, std::min(hit_upper_, value));
    log_odds_[idx] = static_cast<uint8_t>(value);
  }

  void raycastFree(const std::pair<int, int>& start, const std::pair<int, int>& end) {
    int x0 = start.first;
    int y0 = start.second;
    const int x1 = end.first;
    const int y1 = end.second;

    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
      if (!(x0 == x1 && y0 == y1)) {
        applyDelta(x0, y0, free_delta_);
      } else {
        break;
      }
      const int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }
  }

  void publishMap(const ros::Time& stamp) {
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = world_frame_;
    grid.info.resolution = static_cast<float>(grid_size_);
    grid.info.width = static_cast<uint32_t>(width_);
    grid.info.height = static_cast<uint32_t>(height_);
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(static_cast<std::size_t>(width_ * height_), -1);

    for (int i = 0; i < width_ * height_; ++i) {
      if (!known_flags_[i]) {
        grid.data[static_cast<std::size_t>(i)] = -1;
        continue;
      }
      // const int occ = static_cast<int>((static_cast<int>(log_odds_[i]) * 100) / hit_upper_);
      const int occ = static_cast<int>(log_odds_[i]);
      grid.data[static_cast<std::size_t>(i)] = static_cast<int8_t>(std::max(0, std::min(100, occ)));
    }
    map_pub_.publish(grid);
  }

  bool lookupTransformWithFallback(const std::string& target_frame,
                                   const std::string& source_frame,
                                   const ros::Time& stamp,
                                   geometry_msgs::TransformStamped& out) {
    try {
      out = tf_buffer_.lookupTransform(target_frame, source_frame, stamp, ros::Duration(0.05));
      return true;
    } catch (const tf2::TransformException&) {
      // Bag replay / now()-stamped clouds often miss exact-time TF.
    }
    try {
      out = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.05));
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "TF lookup failed (%s -> %s): %s",
                        source_frame.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (msg->width == 0 && msg->height == 0) {
      return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time(0) : msg->header.stamp;
    const std::string cloud_frame = msg->header.frame_id;

    geometry_msgs::TransformStamped tf_world_from_body_msg;
    geometry_msgs::TransformStamped tf_body_from_cloud_msg;
    geometry_msgs::TransformStamped tf_world_from_cloud_msg;

    if (!lookupTransformWithFallback(world_frame_, body_frame_, stamp, tf_world_from_body_msg)) {
      ROS_WARN_THROTTLE(1.0, "Missing TF for world<-body (%s <- %s), cloud frame: %s",
                        world_frame_.c_str(), body_frame_.c_str(), cloud_frame.c_str());
      return;
    }
    if (cloud_frame != body_frame_ &&
        !lookupTransformWithFallback(body_frame_, cloud_frame, stamp, tf_body_from_cloud_msg)) {
      ROS_WARN_THROTTLE(1.0, "Missing TF for body<-cloud (%s <- %s), world frame: %s",
                        body_frame_.c_str(), cloud_frame.c_str(), world_frame_.c_str());
      return;
    }
    if (cloud_frame != world_frame_ &&
        !lookupTransformWithFallback(world_frame_, cloud_frame, stamp, tf_world_from_cloud_msg)) {
      ROS_WARN_THROTTLE(1.0, "Missing TF for world<-cloud (%s <- %s), body frame: %s",
                        world_frame_.c_str(), cloud_frame.c_str(), body_frame_.c_str());
      return;
    }

    const tf2::Transform tf_world_from_body = toTf(tf_world_from_body_msg);
    const tf2::Transform tf_body_from_cloud =
        (cloud_frame == body_frame_) ? tf2::Transform::getIdentity() : toTf(tf_body_from_cloud_msg);
    const tf2::Transform tf_world_from_cloud =
        (cloud_frame == world_frame_) ? tf2::Transform::getIdentity() : toTf(tf_world_from_cloud_msg);

    const tf2::Vector3 body_in_world = tf_world_from_body.getOrigin();
    ensureInside(body_in_world.x(), body_in_world.y());

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    std::unordered_set<std::pair<int, int>, PairHash> obstacle_cells;
    std::unordered_set<std::pair<int, int>, PairHash> ground_cells;

    for (const auto& p : cloud.points) {
      if (!isFinite(p)) {
        continue;
      }

      const tf2::Vector3 p_body_full = tf_body_from_cloud * tf2::Vector3(p.x, p.y, p.z);
      const double range = std::hypot(p_body_full.x(), p_body_full.y());
      if (range > max_range_) {
        continue;
      }

      const tf2::Vector3 p_world_full = tf_world_from_cloud * tf2::Vector3(p.x, p.y, p.z);
      const Point2D p_world{p_world_full.x(), p_world_full.y()};
      const int gx = static_cast<int>(std::floor(p_world.x / grid_size_));
      const int gy = static_cast<int>(std::floor(p_world.y / grid_size_));
      const std::pair<int, int> key{gx, gy};

      // Height is measured from body origin along gravity-aligned world Z.
      const double height_from_body_origin = p_world_full.z() - body_in_world.z();
      if (height_from_body_origin <= z_min_) {
        ground_cells.insert(key);
      } else if (height_from_body_origin <= z_max_) {
        obstacle_cells.insert(key);
      }
    }

    for (const auto& cell : obstacle_cells) {
      ground_cells.erase(cell);
    }

    for (const auto& cell : obstacle_cells) {
      const auto world_center = cellToWorldCenter(cell.first, cell.second);
      ensureInside(world_center.first, world_center.second);
    }
    for (const auto& cell : ground_cells) {
      const auto world_center = cellToWorldCenter(cell.first, cell.second);
      ensureInside(world_center.first, world_center.second);
    }
    ensureInside(body_in_world.x(), body_in_world.y());

    const auto start = worldToCell(body_in_world.x(), body_in_world.y());

    for (const auto& cell : obstacle_cells) {
      const auto world_center = cellToWorldCenter(cell.first, cell.second);
      const auto end = worldToCell(world_center.first, world_center.second);
      raycastFree(start, end);
    }

    for (const auto& cell : ground_cells) {
      const auto world_center = cellToWorldCenter(cell.first, cell.second);
      const auto idx = worldToCell(world_center.first, world_center.second);
      applyDelta(idx.first, idx.second, free_delta_);
    }

    for (const auto& cell : obstacle_cells) {
      const auto world_center = cellToWorldCenter(cell.first, cell.second);
      const auto idx = worldToCell(world_center.first, world_center.second);
      applyDelta(idx.first, idx.second, hit_delta_);
    }

    publishMap(msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher map_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_cloud_topic_;
  std::string body_frame_;
  std::string world_frame_;
  std::string output_map_topic_;

  double z_min_{-0.1};
  double z_max_{0.2};
  double max_range_{15.0};
  double grid_size_{0.1};
  double area_size_{50.0};

  int hit_delta_{15};
  int free_delta_{-5};
  int hit_upper_{128};
  int hit_lower_{0};

  bool save_2d_grid_{true};
  std::string save_dir_;
  std::string save_timestamp_;

  bool initialized_{false};
  int width_{0};
  int height_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};

  std::vector<uint8_t> log_odds_;
  std::vector<uint8_t> known_flags_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "light_terrain_analysis");
  LightTerrainAnalysis node;
  ros::spin();
  node.saveMapIfNeeded();
  return 0;
}
