#include <memory>

#include "OdometryFrame.h"

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace odometry_to_graph
{
OdometryFrame::Ptr OdometryFrame::load(const std::string& cloud_filename, const std::string& pose_filename) {
    std::ifstream ifs(pose_filename);
    if(!ifs) {
        std::cerr << "error : failed to load " << pose_filename << std::endl;
        return nullptr;
    }

    Eigen::Matrix4d mat;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
        ifs >> mat(i, j);
        }
    }

    Eigen::Isometry3d pose(mat);

    return std::make_shared<OdometryFrame>(cloud_filename, pose);
}

OdometryFrame::OdometryFrame(const std::string& raw_cloud_path, const Eigen::Isometry3d& pose, 
    unsigned long stamp_sec, unsigned long stamp_usec) 
    : raw_cloud_path(raw_cloud_path)
    , cloud_(nullptr)
    , pose(pose)
    , stamp_sec(stamp_sec)
    , stamp_usec(stamp_usec)
    , downsample_resolution(0.0) {}

OdometryFrame::OdometryFrame(const std::string& raw_cloud_path, const Eigen::Isometry3d& pose) 
    : raw_cloud_path(raw_cloud_path)
    , cloud_(nullptr)
    , pose(pose)
    , downsample_resolution(0.0f) {
    char underscore;
    std::stringstream sst(boost::filesystem::path(raw_cloud_path).filename().string());
    sst >> stamp_sec >> underscore >> stamp_usec;
}

const pcl::PointCloud<pcl::PointXYZI>::Ptr& OdometryFrame::get_cloud() {
    if (this->cloud_ == nullptr) {
        this->cloud_ = load_cloud(this->raw_cloud_path);
    }

    return this->cloud_;
}

const pcl::PointCloud<pcl::PointXYZI>::Ptr& OdometryFrame::set_resolution(float resolution) {
    if (std::abs(this->downsample_resolution - resolution) < 0.001f)
    {
        return this->cloud_;
    }

    this->downsample_resolution = resolution;

    if(this->downsample_resolution > 0.001f) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud = load_cloud(this->raw_cloud_path);
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setLeafSize(this->downsample_resolution, this->downsample_resolution, this->downsample_resolution);
        voxel_grid.setInputCloud(raw_cloud);
        voxel_grid.filter(*downsampled);

        static bool needWarning = true;
        if (needWarning && downsampled->size() == raw_cloud->size())
        {
            needWarning = false;
            std::cout << "Voxelization: Leaf size is too small for the input dataset. Integer indices would overflow" << std::endl;
        }

        this->cloud_ = downsampled;
    }

    return this->cloud_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OdometryFrame::load_cloud(const std::string& filename) const {
    std::string extension = boost::filesystem::path(filename).extension().string();
    if(extension == ".pcd") {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile(filename, *cloud);
        return cloud;
    } else if (extension == ".txt") {
        std::ifstream ifs(filename);
        if(!ifs) {
            std::cerr << "warning: failed to open " << filename << std::endl;
            return nullptr;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        while(!ifs.eof()) {
            std::string line;
            std::getline(ifs, line);
            if(line.empty()) {
                continue;
            }

            std::stringstream sst(line);

            pcl::PointXYZI pt;
            sst >> pt.x >> pt.y >> pt.z >> pt.intensity;

            cloud->push_back(pt);
        }

        cloud->is_dense = false;
        cloud->width = cloud->size();
        cloud->height = 1;

        return cloud;
    }

    std::cerr << "unknown extension: " << extension << std::endl;
    std::cerr << "input file : " << filename << std::endl;
    abort();

    return nullptr;
}
}