#include <memory>

#include "Graph.h"

#include <g2o/core/robust_kernel_impl.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

namespace merge_map {

  Graph::Graph() {
    this->factory = g2o::Factory::instance();
  }

  bool Graph::load_all(const std::string &directory){
    std::cout << "load_all: " << directory << std::endl;

    return load_graph(directory) && load_keyframes(directory);
  }

  bool Graph::save_all(const std::string &directory)
  {
    std::cout << "save_all" << std::endl;

    save(directory + "/graph.g2o");

    int keyframe_id = 0;
    for(const auto& keyframe : keyframes) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % (keyframe_id++);
      keyframe.second->save(sst.str());
    }

    std::ofstream gpsIdsStream;
    gpsIdsStream.open(directory + "/gps_vertices.txt");

    if (gpsIdsStream.is_open())
    {
      for (auto &[key, value] : gpsVertices)
      {
        if (value)
        {
          gpsIdsStream << key << "\n";
        }
      }
        
      gpsIdsStream.close();
    }

    return true;
  }

  bool Graph::merge(const Graph *const other) {
    std::cout << "merge" << std::endl;
    long max_vertex_id = 0;
    long max_edge_id = 0;

    for(const auto& vertex : graph->vertices()) {
      max_vertex_id = std::max<long>(max_vertex_id, vertex.first);
    }
    for(const auto& edge : graph->edges()) {
      max_edge_id = std::max<long>(max_edge_id, edge->id());
    }

    // clone vertices
    for(const auto& vertex : other->graph->vertices()) {
      long new_vertex_id = ++max_vertex_id;
      auto v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(vertex.second);

      std::stringstream sst;
      if(!v->write(sst)) {
        std::cerr << "error: failed to write vertex data" << std::endl;
        return false;
      }

      auto new_v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(this->factory->construct(this->factory->tag(v)));
      if(!new_v->read(sst)) {
        std::cerr << "error: failed to read vertex data" << std::endl;
        return false;
      }
      new_v->setFixed(v->fixed());
      new_v->setId(new_vertex_id);
      graph->addVertex(new_v);

      const auto iter = other->gpsVertices.find(vertex.first);

      if (iter != other->gpsVertices.end())
      {
        gpsVertices[new_v->id()] = iter->second;
      }

      new_vertices_map[v->id()] = new_v;
    }

    // clone edges
    for(const auto& edge : other->graph->edges()) {
      long new_edge_id = ++max_edge_id;
      auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);

      // copy params via g2o::Factory
      std::stringstream sst;
      if(!e->write(sst)) {
        std::cerr << "error: failed to write edge data" << std::endl;
        return false;
      }

      auto new_e = dynamic_cast<g2o::OptimizableGraph::Edge*>(factory->construct(factory->tag(e)));
      if(!new_e->read(sst)) {
        std::cerr << "error: failed to read edge data" << std::endl;
        return false;
      }
      new_e->setId(new_edge_id);

      // remap vertices with new ones
      for(int i = 0; i < new_e->vertices().size(); i++) {
        new_e->vertices()[i] = new_vertices_map[e->vertices()[i]->id()];
      }

      // copy robust kernel
      if(e->robustKernel()) {
        g2o::RobustKernel* kernel = nullptr;

        if(dynamic_cast<g2o::RobustKernelHuber*>(e->robustKernel())) {
          kernel = new g2o::RobustKernelHuber();
        }

        if(kernel == nullptr) {
          std::cerr << "warning: unknown kernel type!!" << std::endl;
        } else {
          kernel->setDelta(e->robustKernel()->delta());
          new_e->setRobustKernel(kernel);
        }
      }

      edge->setId(new_edge_id);
      graph->addEdge(new_e);
    }

    // copy keyframes
    for(const auto& keyframe : other->keyframes) {
      keyframe.second->node = dynamic_cast<g2o::VertexSE3*>(new_vertices_map[keyframe.second->id()]);
      assert(keyframe.second->node);
      keyframes[keyframe.second->id()] = keyframe.second;
    }

    std::cout << "new gpsVertices size: " << gpsVertices.size() << std::endl;

    return true;
  }

  bool Graph::load_graph(const std::string &directory)
  {
    std::cout << "load_graph: " << directory << std::endl;

    if(!load(directory + "/graph.g2o")) {
      return false;
    }

    std::ifstream gpsVerticesFile(directory + "/gps_vertices.txt");

    if (gpsVerticesFile.is_open())
    {
        long vertexId = -1;

        while (gpsVerticesFile >> vertexId)
        {
            gpsVertices[vertexId] = true;
        }
        gpsVerticesFile.close();
    }

    std::cout << "Gps vertices on the graph: " << gpsVertices.size() << std::endl;

    return true;
  }

  bool Graph::load_keyframes(const std::string& directory) {
    std::cout << "load_keyframes: " << directory << std::endl;
    for(int i = 0;; i++) {
      std::string keyframe_dir = (boost::format("%s/%06d") % directory % i).str();
      if(!boost::filesystem::is_directory(keyframe_dir)) {
        break;
      }

      auto keyframe = std::make_shared<hdl_graph_slam::KeyFrame>(keyframe_dir, graph.get());
      if(!keyframe->node) {
        std::cerr << "error : failed to load keyframe: " << keyframe_dir << std::endl;
      } else {
        keyframes[keyframe->id()] = keyframe;
      }
    }

    return true;
  }

  bool Graph::save_pointcloud(const std::string& filename) {
    std::cout << "save_pointcloud: " << filename << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZI>());

    for(const auto& keyframe : keyframes) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*keyframe.second->cloud, *transformed, keyframe.second->node->estimate().cast<float>());

      std::copy(transformed->begin(), transformed->end(), std::back_inserter(accumulated->points));
    }

    accumulated->is_dense = false;
    accumulated->width = accumulated->size();
    accumulated->height = 1;

    return pcl::io::savePCDFileBinary(filename, *accumulated);
  }

}