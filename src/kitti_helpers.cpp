#include "kitti_helpers.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>


struct PathCompare
{
    bool operator()(const boost::filesystem::path& a, const boost::filesystem::path& b)
    {
        return a.string() < b.string();
    }
};

Eigen::Matrix4f computePoseFromOxts(const KittiOxts_t& oxts, const float scale)
{
    // Earth radius (approx.) in meters.
    static const float earthRadius = 6378137.0f;

    // Compute the rotation transform.
    auto rz = Eigen::Affine3f(Eigen::AngleAxisf(oxts.yaw, Eigen::Vector3f::UnitZ()));
    auto h = rz.matrix();

    // Add in the translation.
    h(0,3) = scale * oxts.lon * kPi * earthRadius / 180.0f;
    h(1,3) = scale * earthRadius * std::log(std::tan(((90.0f + oxts.lat) * kPi) / 360.0f));
    
    return h;
}

std::vector<boost::filesystem::path> getDirectoryContents(
    const boost::filesystem::path& kPath,
    bool findDirs,
    const std::string& fileExt)
{
    std::vector<boost::filesystem::path> dirContents;
    for (auto& item : boost::filesystem::directory_iterator(kPath))
    {
        const boost::filesystem::path& p = item.path();
        if (findDirs && boost::filesystem::is_directory(p))
        {
            dirContents.push_back(p);
        }
        else if (!findDirs && boost::filesystem::is_regular_file(p)
            && p.string().find(fileExt) != std::string::npos)
        {
            dirContents.push_back(p);
        }
    }
    std::sort(dirContents.begin(), dirContents.end(), PathCompare());
    return dirContents;
}

KittiOxts_t readOxtsData(const std::string& kOxtsPath)
{
    // Read the OXTS data from disk.
    std::fstream oxtsfile(kOxtsPath.c_str(), std::ios::in);
    if (!oxtsfile.good())
    {
        std::cerr << "Cannot open OXTS file: " << kOxtsPath << std::endl;
        exit(EXIT_FAILURE);
    }
    oxtsfile.seekg(0, std::ios::beg);
    std::string line;
    std::getline(oxtsfile, line);

    // Create the Kitti OXTS struct.
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(" "));

    // return oxts;
    if (tokens.size() != 30)
    {
        std::cerr << "There should be 30 numbers from file '" << kOxtsPath << "'. Found ";
        std::cerr << tokens.size() << ".\n";
        exit(EXIT_FAILURE);
    }
    return KittiOxts_t(tokens);
}

void readVelodyneBin(
    const std::string& kBinPath,
    pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    // Load the point cloud binary file.
    std::fstream binfile(kBinPath.c_str(), std::ios::in | std::ios::binary);
    if (!binfile.good())
    {
        std::cerr << "Cannot load binary file: " << kBinPath << std::endl;
        exit(EXIT_FAILURE);
    }

    // Iteratively load all the points from the binary file.
    binfile.seekg(0, std::ios::beg);
    for (std::size_t i = 0; binfile.good() && !binfile.eof(); i++)
    {
        pcl::PointXYZI pt;
        binfile.read((char*) &pt.x, 3 * sizeof(float));
        binfile.read((char*) &pt.intensity, sizeof(float));
        cloud.push_back(pt);
    }
    binfile.close();
}
