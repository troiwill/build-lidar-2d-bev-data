#include "bev2d_helpers.hpp"
#include "kitti_helpers.hpp"
#include "point_cloud_dim_2d.hpp"
#include "transform_xyt.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <string>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/range/iterator_range.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace boostpo = boost::program_options;
namespace boostfs = boost::filesystem;


int main(int argc, char** argv)
{
    // Parse the command line arguments.
    std::string basedir, savedir;
    float xlen = 0.f, ylen = 0.f, radius = 0.f, res = 0.f;
    bool useInten = false;

    boostpo::options_description desc("Build Kitti 2D BEV Data Program Options");
    desc.add_options()
        ("help", "Print help message.")
        ("basedir", boostpo::value<std::string>(&basedir)->required(), "Kitti base directory.")
        ("savedir", boostpo::value<std::string>(&savedir)->required(), "Output directory for Kitti learning data.")
        ("res", boostpo::value<float>(&res)->required(), "The resolution of BEV images (in meters).")
        ("xlen", boostpo::value<float>(&xlen), "'x' length of online scan (in meters).")
        ("ylen", boostpo::value<float>(&ylen), "'y' length of online scan (in meters).")
        ("radius", boostpo::value<float>(&radius), "'radius' length of online scan from center (in meters).")
        ("useInten", boostpo::bool_switch(&useInten), "Use intensity values from LiDAR.")
        ;

    boostpo::variables_map vm;
    boostpo::store(boostpo::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(EXIT_SUCCESS);
    }
    boostpo::notify(vm);

    // Form the base and save directory paths, and check if the base directory exists.
    boostfs::path basepath(basedir), savePath(savedir);
    if (!boostfs::exists(basepath))
    {
        std::cerr << "Base directory does not exist: " << basedir << std::endl;
        exit(EXIT_FAILURE);
    }

    // Create a vector containing Kitti sequence information.
    const std::vector<KittiSequence_t> kKittiSeqs = {
        KittiSequence_t("00", "2011_10_03_drive_0027",    0, 4540, true),
        KittiSequence_t("01", "2011_10_03_drive_0042",    0, 1100, true),
        KittiSequence_t("02", "2011_10_03_drive_0034",    0, 4660, true),
        KittiSequence_t("03", "2011_09_26_drive_0067",    0,  800, true),
        KittiSequence_t("04", "2011_09_30_drive_0016",    0,  270, true),
        KittiSequence_t("05", "2011_09_30_drive_0018",    0, 2760, true),
        KittiSequence_t("06", "2011_09_30_drive_0020",    0, 1100, true),
        KittiSequence_t("07", "2011_09_30_drive_0027",    0, 1100, true),
        KittiSequence_t("08", "2011_09_30_drive_0028", 1100, 5170, true),
        KittiSequence_t("09", "2011_09_30_drive_0033",    0, 1590, true),
        KittiSequence_t("10", "2011_09_30_drive_0034",    0, 1200, true)
    };

    std::cout << "Builder parameters:\n";
    std::cout << "* Base directory: " << basedir << "\n* Save directory: " << savedir << std::endl;
    std::cout << "* LiDAR BEV resolution: " << res << " meters\n";
    if (useInten)
        std::cout << "* Using LiDAR intensity values.\n";

    float scanLenX = 0.f, scanLenY = 0.f;
    if (radius > 0.f)
    {
        scanLenX = scanLenY = 2. * radius;
        std::cout << "* Using radius value: " << radius;
    }
    else
    {
        scanLenX = 2. * xlen;
        scanLenY = 2. * ylen;
        std::cout << "* Using x,y lengths (half length): xlen = " << xlen
                  << ", ylen = " << ylen;
    }
    std::cout << "\n\n";
    
    // Iterate over all Kitti date directories in 'datedirs'.
    for (const KittiSequence_t& kSeq : kKittiSeqs)
    {
        std::cout << "Processing Kitti sequence: " << kSeq.id() << " (" << kSeq.name() << ")\n";
        std::cout <<     "Sequence should have " << (kSeq.end() - kSeq.start() + 1) << " scans.\n";

        std::string kittiDate = kSeq.name().substr(0, 10);
        boostfs::path drivepath(basepath / kittiDate / kSeq.name());
        boostfs::path seqSavePath(savePath / kSeq.id());
        
        // Create the save path directory if necessary.
        if (!boostfs::exists(seqSavePath) && !boostfs::create_directories(seqSavePath))
        {
            std::cerr << "Could not create path: " << seqSavePath.string() << std::endl;
            exit(EXIT_FAILURE);
        }

        std::vector<pcl::PointCloud<pcl::PointXYZI>> transformedClouds;
        std::vector<TransformXYTheta> xytTFs;
        std::vector<std::string> velonames;

        Eigen::Vector3f origin;
        float scale = 0.0f;
        bool originIsSet = false;

        for (auto veloId = kSeq.start(); veloId <= kSeq.end(); veloId++)
        {
            std::string veloname = KittiSequence_t::to_string(veloId);
            
            // Read in the Velodyne LiDAR file.
            std::string velofilename = veloname + ".bin";
            boostfs::path velofile(drivepath / "velodyne_points" / "data" / velofilename);
            std::cout << "    Processing Velodyne: " << velofile.filename() << "\r" << std::flush;
            pcl::PointCloud<pcl::PointXYZI> veloCloud;
            readVelodyneBin(velofile.string(), veloCloud);

            // Create a 2D BEV rasterized image for an online scan and save to disk.
            pcl::PointCloud<pcl::PointXYZI> veloCloudROI;
            if (radius == 0)
                extractPointCloudROI(veloCloud, xlen, ylen, veloCloudROI);
            else
                extractPointCloudROI(veloCloud, radius, veloCloudROI);
                
            auto roiDim = PointCloudDim2D::getMinMaxData2D(veloCloudROI);
            translateDataXY(veloCloudROI, Eigen::Vector2f(-scanLenX / 2.f, -scanLenY / 2.f));
            auto onlineBev = buildBEVFromCloud(veloCloudROI, scanLenX, scanLenY, res, useInten);
            writeBGM((seqSavePath / (veloname + ".pgm")).string(), onlineBev);
                
            // Read in the OXTS data with the same LiDAR file name.
            std::string oxtsfile  = (drivepath / "oxts" / "data" / (veloname + ".txt")).string();
            if (!boost::filesystem::is_regular_file(oxtsfile))
            {
                std::cerr << "File does not exist: " << oxtsfile << std::endl;
                exit(EXIT_FAILURE);
            }
            KittiOxts_t oxts = readOxtsData(oxtsfile);

            // Compute the cloud transform into world coordinates.
            if (!originIsSet)
                scale = std::cos(oxts.lat * kPi / 180.0f);
                
            Eigen::Matrix4f toWorldTF = computePoseFromOxts(oxts, scale);
            if (!originIsSet)
            {
                origin = toWorldTF.block<3,1>(0,3);
                originIsSet = true; // Now, mark 'origin' as set.
            }
                
            // Add velodyne cloud and the world transform to vectors.
            toWorldTF.block<3,1>(0,3) -= origin;
            pcl::PointCloud<pcl::PointXYZI> tfdVeloCloud;
            pcl::transformPointCloud(veloCloud, tfdVeloCloud, toWorldTF);

            velonames.push_back(veloname);
            transformedClouds.push_back(tfdVeloCloud);
            xytTFs.push_back(TransformXYTheta(toWorldTF(0,3), toWorldTF(1,3), oxts.yaw));
        }
        // Aggregate the clouds for this drive.
        std::cout << "   Aggregating point clouds to create map..." << std::flush;
        pcl::PointCloud<pcl::PointXYZI> driveCloud;
        aggregatePointClouds(transformedClouds, driveCloud);
        std::cout << "Done!\n" << std::flush;

        // Shift the aggregated point cloud such that the minimum (x,y) is (0,0).
        std::cout << "   Shifting cloud and origin data..." << std::flush;
        auto mapDim = PointCloudDim2D::getMinMaxData2D(driveCloud);
        translateDataXY(driveCloud, mapDim.getCloudMinPt());
        translateDataXY(xytTFs, mapDim.getCloudMinPt());
        std::cout << "Done!\n" << std::flush;

        // Build the 2D BEV map from the aggregated cloud.
        std::cout << "   Saving 2D BEV LiDAR-based map..." << std::flush;
        auto mapBev = buildBEVFromCloud(driveCloud, mapDim.getCloudLenX(),
                                        mapDim.getCloudLenY(), res, useInten);
        writeBGM((seqSavePath / "map.pgm").string(), mapBev);
        std::cout << "Done!\n   Save path: " << seqSavePath.string() << std::flush;

        // Write the information file.
        std::cout << "\n   Writing info file..." << std::flush;
        writeScanInfo((seqSavePath / "info.txt").string(), velonames, xytTFs,
                      std::make_pair(mapBev.rows(), mapBev.cols()), res);
        std::cout << "Done!\n\n" << std::flush;
    }
    exit(EXIT_SUCCESS);
}
