#include "kitti_helpers.hpp"
#include "kitti_oxts.hpp"
#include "kitti_sequence.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

namespace boostfs = boost::filesystem;

using namespace Eigen;
using namespace kitti_bev;
using namespace pcl;
using namespace std;


struct PathCompare
{
    bool operator()(const boostfs::path& a, const boostfs::path& b)
    {
        return a.string() < b.string();
    }
};

Matrix4f kitti_bev::computePoseFromOxts(const KittiOxts_t& oxts, const float scale)
{
    // Earth radius (approx.) in meters.
    static const float earthRadius = 6378137.0f;

    // Compute the rotation transform.
    auto rz = Affine3f(AngleAxisf(oxts.yaw, Vector3f::UnitZ()));
    auto h = rz.matrix();

    // Add in the translation.
    h(0,3) = scale * oxts.lon * kPi * earthRadius / 180.0f;
    h(1,3) = scale * earthRadius * log(tan(((90.0f + oxts.lat) * kPi) / 360.0f));
    
    return h;
}

vector<boostfs::path> kitti_bev::getDirectoryContents(const boostfs::path& kPath, bool findDirs,
    const string& fileExt)
{
    vector<boostfs::path> dirContents;
    for (auto& item : boostfs::directory_iterator(kPath))
    {
        const boostfs::path& p = item.path();
        if (findDirs && boostfs::is_directory(p))
        {
            dirContents.push_back(p);
        }
        else if (!findDirs && boostfs::is_regular_file(p)
            && p.string().find(fileExt) != string::npos)
        {
            dirContents.push_back(p);
        }
    }
    sort(dirContents.begin(), dirContents.end(), PathCompare());
    return dirContents;
}

KittiOxts_t kitti_bev::readOxtsData(const string& kOxtsPath)
{
    // Read the OXTS data from disk.
    fstream oxtsfile(kOxtsPath.c_str(), ios::in);
    if (!oxtsfile.good())
    {
        cerr << "Cannot open OXTS file: " << kOxtsPath << endl;
        exit(EXIT_FAILURE);
    }
    oxtsfile.seekg(0, ios::beg);
    string line;
    getline(oxtsfile, line);

    // Create the Kitti OXTS struct.
    vector<string> tokens;
    boost::split(tokens, line, boost::is_any_of(" "));

    // return oxts;
    if (tokens.size() != 30)
    {
        cerr << "There should be 30 numbers from file '" << kOxtsPath << "'. Found ";
        cerr << tokens.size() << ".\n";
        exit(EXIT_FAILURE);
    }
    return KittiOxts_t(tokens);
}

void kitti_bev::readVelodyneBin(const string& kBinPath, PointCloud<PointXYZI>& cloud)
{
    // Load the point cloud binary file.
    fstream binfile(kBinPath.c_str(), ios::in | ios::binary);
    if (!binfile.good())
    {
        cerr << "Cannot load binary file: " << kBinPath << endl;
        exit(EXIT_FAILURE);
    }

    // Iteratively load all the points from the binary file.
    binfile.seekg(0, ios::beg);
    for (size_t i = 0; binfile.good() && !binfile.eof(); i++)
    {
        pcl::PointXYZI pt;
        binfile.read((char*) &pt.x, 3 * sizeof(float));
        binfile.read((char*) &pt.intensity, sizeof(float));
        cloud.push_back(pt);
    }
    binfile.close();
}
