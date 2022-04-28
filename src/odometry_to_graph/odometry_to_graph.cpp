#include "OdometrySet.h"

#include <memory>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <pcl/console/print.h>

#include <ros/package.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_to_graph");
  ros::NodeHandle nh;

  std::string odom_path;
  std::string save_path;
  double delta_x;
  double delta_angle;
  double downsample_resolution;
  double inf_pos;
  double inf_rot;

  nh.getParam("odom_path", odom_path);
  nh.getParam("save_path", save_path);
  nh.getParam("delta_x", delta_x);
  nh.getParam("delta_angle", delta_angle);
  nh.getParam("downsample_resolution", downsample_resolution);
  nh.getParam("inf_pos", inf_pos);
  nh.getParam("inf_rot", inf_rot);

  if (odom_path.empty())
  {
    std::cerr << "odom_path is empty" << std::endl;
    
    return 0;
  }

  if (save_path.empty())
  {
    std::cerr << "save_path is empty" << std::endl;
    return 0;
  }

  boost::filesystem::remove_all(save_path);
  boost::filesystem::create_directories(save_path);

  if (!boost::filesystem::is_directory(save_path))
  {
    std::cerr << "save_path is not found: " << save_path << std::endl;
    return 0;
  }


  delta_angle *= EIGEN_PI / 180.0f;

  std::cout << "oarameters:" << std::endl;
  std::cout << "odom_path: " << odom_path << std::endl;
  std::cout << "save_path: " << save_path << std::endl;
  std::cout << "delta_x: " << delta_x << std::endl;
  std::cout << "delta_angle: " << delta_angle << std::endl;
  std::cout << "downsample_resolution: " << downsample_resolution << std::endl;
  std::cout << "inf_pos: " << inf_pos << std::endl;
  std::cout << "inf_rot: " << inf_rot << std::endl << std::endl;

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  
  auto odometry_set = std::make_unique<odometry_to_graph::OdometrySet>(downsample_resolution, delta_x, delta_angle, inf_pos, inf_rot);
  odometry_set->load(odom_path);
  odometry_set->save(save_path);

  std::cout << "Done." << std::endl;

  return 0;
}
