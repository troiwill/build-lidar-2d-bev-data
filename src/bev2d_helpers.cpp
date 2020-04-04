#include "bev2d_helpers.hpp"
#include "transform_xyt.hpp"

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

void writeInfo(
    const std::string& KInfofilename,
    const std::vector<std::string>& kVelonames,
    const std::vector<TransformXYTheta>& kTFs,
    const std::size_t kMapImgHeight,
    const std::size_t kMapImgWidth,
    const float kResolution)
{
    // Sanity check.
    assert(!kInfofilename.empty());
    assert(kVelonames.size() == kTFs.size());
    assert(kMapImgHeight > 0 && kMapImgWidth > 0);
    assert(kResolution > 0.f);

    // Open the file.
    std::ofstream infofile(KInfofilename);
    if (!infofile.good())
    {
        std::cerr << "Cannot open or create info file: " << KInfofilename << std::endl;
        exit(EXIT_FAILURE);
    }

    // Write the information to the file.
    infofile << "scanId,py,px,yaw,res\n";
    auto name_it = kVelonames.cbegin();
    auto tf_it = kTFs.cbegin();
    for (; name_it != kVelonames.cend(); ++name_it, ++tf_it)
    {
        auto scanPxLoc = computePixelLoc(kMapImgWidth, kMapImgHeight, tf_it->x, tf_it->y,
                                         kResolution);
        
        infofile << *name_it << "," << scanPxLoc[0] << "," << scanPxLoc[1] << ","
                 << tf_it->yaw << "," << kResolution << std::endl;
    }
    infofile.close();
}
