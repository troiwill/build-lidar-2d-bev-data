#pragma once

#include <cstring>
#include <iomanip>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;


namespace kitti_bev
{
    struct KittiOxts_t;
    struct KittiSequence_t;
    
    const double kPi = 3.14159265359;

    Eigen::Matrix4f computePoseFromOxts(const KittiOxts_t& oxts, const float scale);

    vector<boost::filesystem::path> getDirectoryContents(const boost::filesystem::path& kPath,
        bool findDirs, const string& fileExt="");

    KittiOxts_t readOxtsData(const string& kOxtsPath);

    void readVelodyneBin(const string& kBinPath, pcl::PointCloud<pcl::PointXYZI>& cloud);
}
