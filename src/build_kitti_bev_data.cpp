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

namespace bpo = boost::program_options;


int main(int argc, char** argv)
{
    // Parse the command line arguments.
    std::string basedir, savedir, date;
    float xlen = 0.f, ylen = 0.f, radius = 0.f, res = 0.f;

    bpo::options_description desc("Build Kitti 2D BEV Data Program Options");
    desc.add_options()
        ("help", "Print help message.")
        ("basedir", bpo::value<std::string>(&basedir)->required(), "Kitti base directory.")
        ("savedir", bpo::value<std::string>(&savedir)->required(), "Output directory for Kitti learning data.")
        ("res", bpo::value<float>(&res)->required(), "The resolution of BEV images (in meters).")
        ("xlen", bpo::value<float>(&xlen), "'x' length of online scan (in meters).")
        ("ylen", bpo::value<float>(&ylen), "'y' length of online scan (in meters).")
        ("radius", bpo::value<float>(&radius), "'radius' length of online scan from center (in meters).")
        ("date", bpo::value<std::string>(&date), "Kitti date.")
        ;

    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(EXIT_SUCCESS);
    }
    bpo::notify(vm);

    // Form the base and save directory paths.
    boost::filesystem::path basepath(basedir), savePath(savedir);
    
    // Check if the base directory exists.
    if (!boost::filesystem::exists(basepath))
    {
        std::cerr << "Base directory does not exist: " << basedir << std::endl;
        exit(EXIT_FAILURE);
    }

    // Should the program use all the Kitti dates or a date that the user specified?
    std::vector<std::string> datedirs;
    if (date.empty())
    {
        datedirs = std::vector<std::string>(
            { "2011_09_26", "2011_09_28", "2011_09_29", "2011_09_30", "2011_10_03" });
    }
    else
    {
        datedirs.push_back(date);
    }
    
    float scanLenX = 0.f, scanLenY = 0.f;
    if (radius > 0.f)
    {
        scanLenX = scanLenY = 2. * radius;
    }
    else
    {
        scanLenX = 2. * xlen;
        scanLenY = 2. * ylen;
    }
    
    // Iterate over all Kitti date directories in 'datedirs'.
    for (auto dated : datedirs )
    {
        std::cout << "Collecting directories for date: " << dated;
        auto drivepaths = getDirectoryContents(basepath / dated, true);
        std::cout << ". Found " << drivepaths.size() << " directories.\n";
        
        // Iterate over all drive directories within a Kitti date directory.
        for (auto drived : drivepaths)
        {
            std::cout << "Collecting Velodyne scans from drive: " << drived.filename();
            auto velodynepaths = getDirectoryContents(
                drived / "velodyne_points" / "data", false, ".bin");
            std::cout << ". Found " << velodynepaths.size() << " directories.\n";
            
            if (velodynepaths.size() == 0)
            {
                std::cout << "   " << drived.filename()
                          << " does not contain any Velodyne BIN files? Skipping...\n\n";
                continue;
            }

            // Form drive save path and create the directory.
            boost::filesystem::path driveSavePath(savePath / dated / drived.filename());
            if (!boost::filesystem::exists(driveSavePath) &&
                !boost::filesystem::create_directories(driveSavePath))
            {
                std::cerr << "Could not create path: " << driveSavePath.string() << std::endl;
                exit(EXIT_FAILURE);
            }

            // Check if this directory might have been processed already.
            if (getDirectoryContents(driveSavePath, false, "pgm").size() == (velodynepaths.size() + 1) &&
                getDirectoryContents(driveSavePath, false, "txt").size() == 1)
            {
                std::cout << "   Seems like " << drived.filename()
                          << " was already processed. Continuing...\n\n";
                continue;
            }

            std::vector<pcl::PointCloud<pcl::PointXYZI>> transformedClouds;
            std::vector<TransformXYTheta> xytTFs;
            std::vector<std::string> velonames;

            Eigen::Vector3f origin;
            float scale = 0.0f;
            bool originIsSet = false;
            
            for (auto velofile : velodynepaths)
            {
                // Read in the Velodyne LiDAR file.
                std::string veloname = velofile.filename().string().substr(0, 10);
		        velonames.push_back(veloname);
                std::cout << "    Processing Velodyne: " << velofile.filename()
                          << "\r" << std::flush;
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
                auto onlineBev = buildBEVFromCloud(veloCloudROI, scanLenX, scanLenY, res);
                writeBGM((driveSavePath / (veloname + ".pgm")).string(), onlineBev);
                
                // Read in the OXTS data with the same LiDAR file name.
                std::string oxtsfile  = (drived / "oxts" / "data" / (veloname + ".txt")).string();
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
                                            mapDim.getCloudLenY(), res);
            writeBGM((driveSavePath / "map.pgm").string(), mapBev);
            std::cout << "Done!\n   Save path: " << driveSavePath.string() << std::flush;

            // Write the information file.
	        std::cout << "\n   Writing info file..." << std::flush;
            writeScanInfo((driveSavePath / "info.txt").string(), velonames, xytTFs,
                          std::make_pair(mapBev.rows(), mapBev.cols()), res);
	        std::cout << "Done!\n\n" << std::flush;
        }
    }
    exit(EXIT_SUCCESS);
}
