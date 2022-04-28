#include "OdometrySet.h"

#include <memory>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace odometry_to_graph
{
static const std::string VERTEX_LABEL = "VERTEX_SE3:QUAT";
static const std::string EDGE_LABEL = "EDGE_SE3:QUAT";

OdometrySet::OdometrySet(const float resolution, const float delta_x, const float delta_angle, 
    const float inf_pos, const float inf_rot)
    : resolution(resolution)
    , delta_x(delta_x)
    , delta_angle(delta_angle)
    , inf_pos(inf_pos)
    , inf_rot(inf_rot) {}

void OdometrySet::load(const std::string& directory) {
    std::cout << "load keyframes: " << directory << std::endl;

    boost::filesystem::directory_iterator itr(directory);
    boost::filesystem::directory_iterator end;

    std::vector<std::string> filenames;
    for(itr; itr != end; itr++) {
        if(itr->path().extension() != ".pcd") {
        continue;
        }

        std::string odom_filename = itr->path().parent_path().string() + "/" + itr->path().stem().string() + ".odom";

        if(!boost::filesystem::exists(odom_filename)) {
        continue;
        }

        filenames.push_back(itr->path().stem().string());
    }

    std::sort(filenames.begin(), filenames.end());

    for(const auto& filename : filenames) {
        auto frame = OdometryFrame::load(directory + "/" + filename + ".pcd", directory + "/" + filename + ".odom");
        if(frame == nullptr) {
        continue;
        }
        frame->set_resolution(resolution);

        if (keyframes.size() == 0) {
            keyframes.push_back(frame);
        } else {
            const auto& last_keyframe_pose = keyframes.back()->pose;
            const auto& current_frame_pose = frame->pose;

            Eigen::Isometry3d delta = last_keyframe_pose.inverse() * current_frame_pose;
            double dx = delta.translation().norm();
            double da = Eigen::AngleAxisd(delta.linear()).angle();

            if(dx > delta_x || da > delta_angle) {
                keyframes.push_back(frame);
            }
        }
    }
}

bool OdometrySet::save(const std::string& directory) {
    std::cout << "save keyframes: " << directory << std::endl;
    std::cout << "keyframes size: " << keyframes.size() << std::endl;

    if (keyframes.empty()) {
        return false;
    }

    if (!save_graph(directory + "/graph.g2o") || !save_keyframes(directory)) {
        return false;
    }

    return true;
    }

bool OdometrySet::save_graph(const std::string& filename) const {
    std::ofstream ofs(filename);
    if(!ofs) {
        return false;
    }

    for(int i = 0; i < keyframes.size(); i++) {
        std::unique_ptr<g2o::VertexSE3> v(new g2o::VertexSE3());
        v->setEstimate(keyframes[i]->pose);

        ofs << VERTEX_LABEL << " " << i << " ";
        v->write(ofs);
        ofs << std::endl;
    }

    for(int i = 0; i < keyframes.size() - 1; i++) {
        const auto& delta_pose = keyframes[i]->pose.inverse() * keyframes[i + 1]->pose;
        std::unique_ptr<g2o::EdgeSE3> e(new g2o::EdgeSE3());
        e->setMeasurement(delta_pose);

        Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
        inf.block<3, 3>(0, 0) *= inf_pos;
        inf.block<3, 3>(3, 3) *= inf_rot;
        e->setInformation(inf);

        ofs << EDGE_LABEL << " " << i << " " << i + 1 << " ";
        e->write(ofs);

        ofs << std::endl;
    }

    ofs.close();

    return true;
}

bool OdometrySet::save_keyframes(const std::string& directory) const {
    for(int i = 0; i < keyframes.size(); i++) {
        std::string keyframe_directory = (boost::format("%s/%06d") % directory % i).str();
        boost::filesystem::create_directories(keyframe_directory);

        boost::filesystem::copy_file(keyframes[i]->raw_cloud_path, keyframe_directory + "/raw.pcd");

        auto cloud = keyframes[i]->get_cloud();

        if (cloud)
        {
            pcl::io::savePCDFileBinary(keyframe_directory + "/cloud.pcd", *cloud);
        }

        std::ofstream ofs(keyframe_directory + "/data");

        if(!ofs) {
            return false;
        }

        ofs << "stamp " << keyframes[i]->stamp_sec << " " << keyframes[i]->stamp_usec << std::endl;
        ofs << "estimate" << std::endl << keyframes[i]->pose.matrix() << std::endl;
        ofs << "odom " << std::endl << keyframes[i]->pose.matrix() << std::endl;
        ofs << "id " << i << std::endl;
    }

    return true;
}

}