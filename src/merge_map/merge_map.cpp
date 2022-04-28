#include "Graph.h"

#include <memory>
#include <boost/filesystem.hpp>

#include <ros/package.h>
#include <ros/ros.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(map_1_path, "", "Description");
DEFINE_string(map_2_path, "", "Description");
DEFINE_string(save_path, "", "Description");
DEFINE_int32(export_pcd, 0, "Description");

int main(int argc, char** argv) {
  ros::init(argc, argv, "merge_maps");
  ros::NodeHandle nh;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_map_1_path.empty()) << "-map_1_path is missing.";
  CHECK(!FLAGS_map_2_path.empty()) << "-map_2_path is missing.";
  CHECK(!FLAGS_save_path.empty()) << "-save_path is missing.";

  boost::filesystem::remove_all(FLAGS_save_path);
  boost::filesystem::create_directories(FLAGS_save_path);

  if (!boost::filesystem::is_directory(FLAGS_save_path))
  {
    std::cerr << "save_path is not found: " << FLAGS_save_path << std::endl;
    return 0;
  }

  std::cout << "parameters:" << std::endl;
  std::cout << "map_1_path: " << FLAGS_map_1_path << std::endl;
  std::cout << "map_2_path: " << FLAGS_map_2_path << std::endl;
  std::cout << "save_path: " << FLAGS_save_path << std::endl;
  std::cout << "export_pcd: " << FLAGS_export_pcd << std::endl << std::endl;

  auto graph1 = std::make_unique<merge_map::Graph>();
  graph1->load_all(FLAGS_map_1_path);

  auto graph2 = std::make_unique<merge_map::Graph>();
  graph2->load_all(FLAGS_map_2_path);

  graph1->merge(graph2.get());
  graph1->save_all(FLAGS_save_path);

  if (FLAGS_export_pcd == 1)
  {
    graph1->save_pointcloud(FLAGS_save_path + "/merged_cloud.pcd");
  }
  std::cout << "Done." << std::endl;

  return 0;
}
