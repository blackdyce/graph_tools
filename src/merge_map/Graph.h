#pragma once

#include <g2o/core/factory.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>

namespace merge_map {

class Graph : protected hdl_graph_slam::GraphSLAM {
  public:
    Graph();

  public:
    bool load_all(const std::string &directory);
    bool save_all(const std::string &directory);
    bool merge(const Graph *const other);

    bool save_pointcloud(const std::string& filename);

  private:
    bool load_graph(const std::string &directory);
    bool load_keyframes(const std::string& directory);

  private:
    g2o::Factory* factory;
    std::unordered_map<long, g2o::HyperGraph::Vertex*> new_vertices_map;
    std::unordered_map<long, hdl_graph_slam::KeyFrame::Ptr> keyframes;
    std::unordered_map<long, bool> gpsVertices;
};

}