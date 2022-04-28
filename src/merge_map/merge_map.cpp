#include "Graph.h"

#include <memory>
#include <boost/filesystem.hpp>

#include <ros/package.h>
#include <ros/ros.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "merge_maps");
  ros::NodeHandle nh;

  std::string map_1_path;
  std::string map_2_path;
  std::string save_path;

  nh.getParam("map_1_path", map_1_path);
  nh.getParam("map_2_path", map_2_path);
  nh.getParam("save_path", save_path);

  if (map_1_path.empty())
  {
    std::cerr << "map_1_path is empty" << std::endl;
    
    return 0;
  }

  if (map_2_path.empty())
  {
    std::cerr << "map_2_path is empty" << std::endl;
    
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

  std::cout << "parameters:" << std::endl;
  std::cout << "map_1_path: " << map_1_path << std::endl;
  std::cout << "map_2_path: " << map_2_path << std::endl;
  std::cout << "save_path: " << save_path << std::endl;

  auto graph1 = std::make_unique<merge_map::Graph>();
  graph1->load_all(map_1_path);

  auto graph2 = std::make_unique<merge_map::Graph>();
  graph2->load_all(map_2_path);

  graph1->merge(graph2.get());
  graph1->save_all(save_path);

  std::cout << "Done." << std::endl;

  return 0;
}
