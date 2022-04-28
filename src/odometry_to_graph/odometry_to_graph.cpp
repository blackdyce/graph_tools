#include "OdometrySet.h"

#include <memory>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <pcl/console/print.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(odom_path, "", "Description");
DEFINE_string(save_path, "", "Description");
DEFINE_double(delta_x, 1, "Description");
DEFINE_double(delta_angle, 15, "Description");
DEFINE_double(resolution, 0.3, "Description");
DEFINE_double(inf_pos, 10, "Description");
DEFINE_double(inf_rot, 20, "Description");

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_to_graph");
  ros::NodeHandle nh;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_odom_path.empty()) << "-odom_path is missing.";
  CHECK(!FLAGS_save_path.empty()) << "-save_path is missing.";
  
  boost::filesystem::remove_all(FLAGS_save_path);
  boost::filesystem::create_directories(FLAGS_save_path);

  if (!boost::filesystem::is_directory(FLAGS_save_path))
  {
    std::cerr << "save_path is not found: " << FLAGS_save_path << std::endl;
    return 0;
  }

  FLAGS_delta_angle *= EIGEN_PI / 180.0f;

  std::cout << "oarameters:" << std::endl;
  std::cout << "odom_path: " << FLAGS_odom_path << std::endl;
  std::cout << "save_path: " << FLAGS_save_path << std::endl;
  std::cout << "delta_x: " << FLAGS_delta_x << std::endl;
  std::cout << "delta_angle: " << FLAGS_delta_angle << std::endl;
  std::cout << "resolution: " << FLAGS_resolution << std::endl;
  std::cout << "inf_pos: " << FLAGS_inf_pos << std::endl;
  std::cout << "inf_rot: " << FLAGS_inf_rot << std::endl << std::endl;

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  
  auto odometry_set = std::make_unique<odometry_to_graph::OdometrySet>(FLAGS_resolution, 
    FLAGS_delta_x, FLAGS_delta_angle, FLAGS_inf_pos, FLAGS_inf_rot);
  odometry_set->load(FLAGS_odom_path);
  odometry_set->save(FLAGS_save_path);

  std::cout << "Done." << std::endl;

  return 0;
}
