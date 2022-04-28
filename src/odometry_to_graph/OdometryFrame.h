#pragma once

#include <memory>

#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace odometry_to_graph
{
struct OdometryFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<OdometryFrame>;

public:
  OdometryFrame(const std::string& raw_cloud_path, const Eigen::Isometry3d& pose, 
    unsigned long stamp_sec, unsigned long stamp_usec);

  OdometryFrame(const std::string& raw_cloud_path, const Eigen::Isometry3d& pose);

public:
  static OdometryFrame::Ptr load(const std::string& cloud_filename, const std::string& pose_filename);

  const pcl::PointCloud<pcl::PointXYZI>::Ptr& get_cloud();

  const pcl::PointCloud<pcl::PointXYZI>::Ptr& set_resolution(float downsample_resolution);

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr load_cloud(const std::string& filename) const;

public:
  // TODO: Getter
  unsigned long stamp_sec;
  unsigned long stamp_usec;
  std::string raw_cloud_path;
  Eigen::Isometry3d pose;

private:
  float downsample_resolution;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
};
}