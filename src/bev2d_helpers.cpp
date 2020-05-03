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

    float x, y, z, inten;
    for (auto p_it = kCloud.begin(); p_it != kCloud.end(); ++p_it)
    {
        auto p = *p_it;
        x = static_cast<float>(p.x);
        y = static_cast<float>(p.y);
        z = static_cast<float>(p.z);
        inten = static_cast<float>(p.intensity);

        pcdf.write((char*)&x, sizeof(float));
        pcdf.write((char*)&y, sizeof(float));
        pcdf.write((char*)&z, sizeof(float));
        pcdf.write((char*)&inten, sizeof(float));
    }
    pcdf.close();
}

void bev2d::writeVelodyneData(const boostfs::path& kSavePath, const vector<VelodyneData_t>& kData)
{
    // Sanity check.
    assert(kData.size() > 0);

    // Open the file.
    string infofilename((kSavePath / "info.txt").string());
    ofstream infofile(infofilename);
    if (!infofile.good())
    {
        cerr << "Cannot open or create info file: " << infofilename << endl;
        exit(EXIT_FAILURE);
    }

    // Write the information to the file.
    float accDistance = 0.f, delDistance = 0.f;
    float prev_tf_x = kData[0].xytheta().x, prev_tf_y = kData[0].xytheta().y;
    float tf_x = 0.f, tf_y = 0.f;
    const int kBufferSize = 250;
    char infostr[kBufferSize];

    snprintf(infostr, kBufferSize, "%10s, %10s, %10s, %10s, %12s, %12s\n",
        "scanId", "scx", "scy", "yaw", "delta_dist", "acc_dist");
    infofile << infostr;

    for (auto data_it = kData.cbegin(); data_it != kData.cend(); ++data_it)
    {
        const VelodyneData_t& cdata = *data_it;
        cout << "Progressing scan '" << cdata.name() << "'.\r";

        // Write the PCD to disk.
        bev2d::writePCDbin((kSavePath / cdata.name()).string() + ".bin", *(cdata.scan()));

        // Compute the pixel location of this scan within the map.
        tf_x = cdata.xytheta().x;
        tf_y = cdata.xytheta().y;

        // Compute delta distance (between this scan and the previous) and accumulated distance.
        delDistance = eucddist(prev_tf_x, prev_tf_y, tf_x, tf_y);
        accDistance += delDistance;

        // Write info to disk.
        snprintf(infostr, kBufferSize, "%10s, %10f, %10f, %10f, %12f, %12f\n",
            cdata.name().c_str(), tf_x, tf_y, cdata.xytheta().yaw, delDistance, accDistance);
        infofile << infostr;

        prev_tf_x = tf_x;
        prev_tf_y = tf_y;
    }
    infofile.close();
}
