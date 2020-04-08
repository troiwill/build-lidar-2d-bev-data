#include "bev2d_helpers.hpp"
#include "transform_xyt.hpp"

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>


void writeBGM(
    const std::string& kSavePath,
    const RMatrixXui8& kMat)
{
    // Sanity checks.
    assert(!kSavePath.empty());
    assert(kMat.rows() > 0 && kMat.cols() > 0);
    
    // Open an output binary stream.
    std::ofstream ofs(kSavePath);
    if (!ofs.good())
    {
        std::cerr << "Could not open the file path: " << kSavePath << std::endl;
        exit(EXIT_FAILURE);
    }

    // Write the PGM file.
    ofs << "P5\n" << kMat.cols() << " " << kMat.rows() << "\n255\n";
    auto v = kMat.data();
    for (std::size_t i = 0; i < kMat.size(); i++)
        ofs << static_cast<std::uint8_t>(v[i]);
    ofs.close();
}

void writeScanInfo(
    const std::string& KInfofilename,
    const std::vector<std::string>& kVelonames,
    const std::vector<TransformXYTheta>& kTFs,
    const std::pair<std::uint32_t, std::uint32_t>& kMapImgSize,
    const float kResolution)
{
    // Sanity check.
    assert(!kInfofilename.empty());
    assert(kVelonames.size() == kTFs.size() && kTFs.size() > 0);
    assert(kMapImgSize.first > 0 && kMapImgSize.second > 0);
    assert(kResolution > 0.f);

    // Open the file.
    std::ofstream infofile(KInfofilename);
    if (!infofile.good())
    {
        std::cerr << "Cannot open or create info file: " << KInfofilename << std::endl;
        exit(EXIT_FAILURE);
    }

    // Write the information to the file.
    float accDistance = 0.f, delDistance = 0.f;
    float prev_tf_x = kTFs[0].x, prev_tf_y = kTFs[0].y;
    float tf_x = 0.f, tf_y = 0.f;
    const int mapHeight = kMapImgSize.first, mapWidth = kMapImgSize.second;
    const int kBufferSize = 150;
    char infostr[kBufferSize];

    snprintf(infostr, kBufferSize, "%10s, %6s, %6s, %10s, %6s, %12s, %12s\n",
             "scanId", "cpy", "cpx", "yaw", "res", "delta_dist", "acc_dist");
    infofile << infostr;
    
    auto name_it = kVelonames.cbegin();
    auto tf_it = kTFs.cbegin();
    for (; name_it != kVelonames.cend(); ++name_it, ++tf_it)
    {
        tf_x = tf_it->x;
        tf_y = tf_it->y;

        // Compute the pixel location of this scan within the map.
        auto scanPxLoc = computePixelLoc(mapHeight, mapWidth, tf_x, tf_y, kResolution);
        
        // Compute delta distance (between this scan and the previous) and accumulated distance.
        delDistance = eucddist(prev_tf_x, prev_tf_y, tf_x, tf_y);
        accDistance += delDistance;
        
        // Write info to disk.
        snprintf(infostr, kBufferSize, "%10s, %6i, %6i, %10f, %6.3f, %12f, %12f\n",
                 name_it->c_str(), scanPxLoc[0], scanPxLoc[1], tf_it->yaw, kResolution,
                 delDistance, accDistance);
        infofile << infostr;
        
        prev_tf_x = tf_x;
        prev_tf_y = tf_y;
    }
    infofile.close();
}
