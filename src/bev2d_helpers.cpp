#include "bev2d_helpers.hpp"
#include "transform_xyt.hpp"
#include "velodyne_data.hpp"

#include <cstdint>
#include <cstdio>
#include <Eigen/Core>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <string>
#include <vector>

namespace boostfs = boost::filesystem;

using namespace bev2d;
using namespace pcl;
using namespace std;


void bev2d::writeBGM(const string& kSavePath, const RMatrixXui8& kMat)
{
    // Sanity checks.
    assert(!kSavePath.empty());
    assert(kMat.rows() > 0 && kMat.cols() > 0);

    // Open an output binary stream.
    ofstream ofs(kSavePath);
    if (!ofs.good())
    {
        cerr << "Could not open the file path: " << kSavePath << endl;
        exit(EXIT_FAILURE);
    }

    // Write the PGM file.
    ofs << "P5\n" << kMat.cols() << " " << kMat.rows() << "\n255\n";
    auto v = kMat.data();
    for (size_t i = 0; i < kMat.size(); i++)
        ofs << static_cast<uint8_t>(v[i]);
    ofs.close();
}

void bev2d::writePCDbin(const string& kPcdSavePath, const PointCloud<PointXYZI>& kCloud)
{
    // Open the file.
    fstream pcdf(kPcdSavePath, ios::out | ios::binary);
    if (!pcdf.good())
    {
        cerr << "Cannot open or create PCD binary file: " << kPcdSavePath << endl;
        exit(EXIT_FAILURE);
    }

    for (auto p_it = kCloud.begin(); p_it != kCloud.end(); ++p_it)
        writeBinToStream(pcdf, *p_it);
    pcdf.close();
}

void bev2d::writePCDbin(const string& kPcdSavePath, const vector<VelodyneData_t>& kData)
{
    // Open the file.
    fstream pcdf(kPcdSavePath, ios::out | ios::binary);
    if (!pcdf.good())
    {
        cerr << "Cannot open or create PCD binary file: " << kPcdSavePath << endl;
        exit(EXIT_FAILURE);
    }

    for (auto d_it = kData.cbegin(); d_it != kData.cend(); ++d_it)
    {
        const PointCloud<PointXYZI>& kCloud = *(d_it->scan());
        for (auto p_it = kCloud.begin(); p_it != kCloud.end(); ++p_it)
            writeBinToStream(pcdf, *p_it);
    }
    pcdf.close();
}

inline
void bev2d::writeBinToStream(fstream& os, const PointXYZI& kPoint)
{
    float x = kPoint.x, y = kPoint.y, z = kPoint.z, intensity = kPoint.intensity;

    os.write((char*)&x, sizeof(float));
    os.write((char*)&y, sizeof(float));
    os.write((char*)&z, sizeof(float));
    os.write((char*)&intensity, sizeof(float));
}
