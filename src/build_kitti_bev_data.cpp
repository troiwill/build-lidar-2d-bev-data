#include "bev2d_helpers.hpp"
#include "kitti_helpers.hpp"
#include "kitti_oxts.hpp"
#include "kitti_sequence.hpp"
#include "point_cloud_dim_2d.hpp"
#include "transform_xyt.hpp"
#include "velodyne_data.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/range/iterator_range.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace boostpo = boost::program_options;
namespace boostfs = boost::filesystem;

using namespace bev2d;
using namespace kitti_bev;
using namespace pcl;
using namespace std;


void loadVelodyneData(const boostfs::path& kDrivepath, const KittiSequence_t& kSeq,
    vector<VelodyneData_t>& data)
{
    cout << "Loading Velodyne data for sequence " << kSeq.id() << endl;

    // Get the transformation matrix going from Velodyne coordinates to IMU.
    const Eigen::Matrix4f kVeloToImuTF = kSeq.getVeloToImuTransform();
    cout << "Velodyne to IMU transform:\n" << kVeloToImuTF << endl << endl;

    // Read in the OXTS data for the first scan in the sequence.
    string firstScanName(KittiSequence_t::to_string(kSeq.start()) + ".txt");
    KittiOxts_t oxts = readOxtsData((kDrivepath / "oxts" / "data" / firstScanName).string());
    const float kScale = cos(oxts.lat * kPi / 180.0f);
    const Eigen::Vector3f kOrigin = (computePoseFromOxts(oxts, kScale)
        * kVeloToImuTF).block<3,1>(0,3);
    cout << "\nMap Origin in world (earth) coordinates:\n" << kOrigin << endl << endl;

    // Launch threads to load the Velodyne data into memory.
    size_t kNumScans = kSeq.getNumScans();
    data.reserve(kNumScans);

    for (size_t veloId = kSeq.start(), i = 1; veloId <= kSeq.end(); veloId++, i++)
    {
        string veloname = KittiSequence_t::to_string(veloId);
        printf("Loading velodyne data... (name: %s) - Progress -> %6.2f%%\r", veloname.c_str(),
        static_cast<float>(100.0 * i / kNumScans));
        cout << flush;

        VelodyneData_t currd;

        // Read in the Velodyne LiDAR file.
        string velofilename = veloname + ".bin";
        boostfs::path velofile(kDrivepath / "velodyne_points" / "data" / velofilename);
        PointCloud<PointXYZI>::Ptr scan(new PointCloud<PointXYZI>());
        readVelodyneBin(velofile.string(), *scan);

        // Read in the vehicle ground truth pose file.
        string oxtsfile = (kDrivepath / "oxts" / "data" / (veloname + ".txt")).string();
        oxts = readOxtsData(oxtsfile);

        // Compute velodyne to world transformation matrix.
        Eigen::Matrix4f toWorldTF = computePoseFromOxts(oxts, kScale) * kVeloToImuTF;
        toWorldTF.block<3,1>(0,3) -= kOrigin;

        // Transform the point cloud.
        pcl::transformPointCloud(*scan, *(currd.scan()), toWorldTF);

        currd.name() = veloname;
        currd.toWorldTf() = toWorldTF;
        currd.xytheta() = TransformXYTheta(toWorldTF(0,3), toWorldTF(1,3), oxts.yaw);

        data.push_back(currd);
    }
    cout << "\nDone!\n\n" << flush;
}

void writeVelodyneData(const vector<VelodyneData_t>& kVelodata, const size_t kNumScansToAgg,
    const KittiSequence_t& kSeq, const boostfs::path& kSavePath)
{
    // Sanity checks.
    assert(kVelodata.size() > 0);
    assert(kNumScansToAgg >= 1);
    
    if (kNumScansToAgg > 1)
    {
        cout << "Aggregating and writing velodyne data to disk. Each cloud has "
             << kNumScansToAgg << " scans.\n";
    }
    else
    {
        cout << "Simply writing velodyne data to disk.\n";
    }

    // Write the information to the file.
    size_t dataIdx = kNumScansToAgg - 1;
    float accDistance = 0.f;
    float delDistance = 0.f;

    float prev_tf_x = kVelodata[dataIdx].xytheta().x;
    float prev_tf_y = kVelodata[dataIdx].xytheta().y;

    float tf_x = 0.f;
    float tf_y = 0.f;

    const int kBufferSize = 250;
    char infostr[kBufferSize];

    // Open the file.
    string infofilename((kSavePath / "info.txt").string());
    ofstream infofile(infofilename);
    if (!infofile.good())
    {
        cerr << "Cannot open or create info file: " << infofilename << endl;
        exit(EXIT_FAILURE);
    }
    snprintf(infostr, kBufferSize, "%10s, %10s, %10s, %10s, %12s, %12s\n",
        "scanId", "scx", "scy", "yaw", "delta_dist", "acc_dist");
    infofile << infostr;

    // Iteratively create the point clouds (if necessary) and write their data to disk.
    const size_t kAggcap = kSeq.getNumScans() - kNumScansToAgg + 1;
    for (size_t i = 1; dataIdx < kSeq.getNumScans(); i++, dataIdx++)
    {
        // Extract the current velodyne data and create the write path for the point cloud.
        const VelodyneData_t& kCdata = kVelodata[dataIdx];
        const string kWritePath((kSavePath / kCdata.name()).string() + ".bin");

        printf("Processing data for %s: Progress -> %6.2f%%\r",
            kCdata.name().c_str(), static_cast<float>(100.0 * i / kAggcap));
        cout << flush;

        // Aggregate the point clouds if necessary.
        if (kNumScansToAgg > 1)
        {
            PointCloud<PointXYZI> cloudToWrite;
            for (size_t offset = 0; offset < kNumScansToAgg; offset++)
                cloudToWrite += *(kVelodata[dataIdx - offset].scan());
            
            // Write the cloud to disk.
            writePCDbin(kWritePath, cloudToWrite);
        }
        else
        {
            // Write the single cloud to disk.
            writePCDbin(kWritePath, *(kCdata.scan()));
        }

        // Compute the pixel location of this scan within the map.
        tf_x = kCdata.xytheta().x;
        tf_y = kCdata.xytheta().y;

        // Compute delta distance (between this scan and the previous) and accumulated distance.
        delDistance = eucddist(prev_tf_x, prev_tf_y, tf_x, tf_y);
        accDistance += delDistance;

        // Write info to disk.
        snprintf(infostr, kBufferSize, "%10s, %10f, %10f, %10f, %12f, %12f\n",
            kCdata.name().c_str(), tf_x, tf_y, kCdata.xytheta().yaw, delDistance, accDistance);
        infofile << infostr;

        prev_tf_x = tf_x;
        prev_tf_y = tf_y;
    }
    infofile.close();
    cout << "\nDone!\n";
}

int main(int argc, char** argv)
{
    // Parse the command line arguments.
    string basedir, savedir;
    int seqid = 0, numScansToAgg = 0, numThreads = 1;

    boostpo::options_description desc("Build Kitti 2D BEV Data Program Options");
    desc.add_options()
        ("help", "Print help message.")
        ("basedir", boostpo::value<string>(&basedir)->required(), "Kitti base directory.")
        ("savedir", boostpo::value<string>(&savedir)->required(), "Output directory for Kitti learning data.")
        ("seqid", boostpo::value<int>(&seqid)->required(), "The Kitti sequence ID (e.g., 09).")
        ("nscans", boostpo::value<int>(&numScansToAgg)->required(), "Number of scans to aggregate.")
        ;

    boostpo::variables_map vm;
    boostpo::store(boostpo::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help"))
    {
        cout << desc << endl;
        exit(EXIT_SUCCESS);
    }
    boostpo::notify(vm);

    // Form the base and save directory paths, and check if the base directory exists.
    boostfs::path basepath(basedir), savePath(savedir);
    if (!boostfs::exists(basepath))
    {
        cerr << "Base directory does not exist: " << basedir << endl;
        exit(EXIT_FAILURE);
    }

    // Print command-line argument information.
    cout << "Builder parameters:\n";
    cout << "* Sequence ID:            " << seqid << endl;
    cout << "* Base directory:         " << basedir << endl;
    cout << "* Save directory:         " << savedir << endl;
    numScansToAgg = max(numScansToAgg, 1);
    cout << "* Num scans to aggregate: " << numScansToAgg << "\n\n";

    // Extract the appropriate Kitti sequence.
    const KittiSequence_t& kSeq = KittiSequence_t::getSeqInfo(seqid);
    cout << "Processing sequence: " << kSeq.id() << " (raw data = " << kSeq.name() << ")\n";
    cout << "Sequence should have " << kSeq.getNumScans() << " scans.\n\n";

    string kittiDate = kSeq.name().substr(0, 10);
    boostfs::path drivepath(basepath / kittiDate / kSeq.name());
    boostfs::path seqSavePath(savePath / kSeq.id());

    // Create the save path directory if necessary.
    if (!boostfs::exists(seqSavePath) && !boostfs::create_directories(seqSavePath))
    {
        cerr << "Could not create path: " << seqSavePath.string() << endl;
        exit(EXIT_FAILURE);
    }

    // Load the velodyne data.
    vector<VelodyneData_t> velodata;
    loadVelodyneData(drivepath, kSeq, velodata);

    // Aggregate all Velodyne scans into a map and write the map to disk.
    cout << "Writing the map point cloud to disk..." << flush;
    writePCDbin((seqSavePath / "map.bin").string(), velodata);
    cout << "Done!\n\n" << flush;

    // Aggregate and write the Velodyne data to disk.
    writeVelodyneData(velodata, numScansToAgg, kSeq, seqSavePath);

    cout << "\nSequence " << kSeq.id() << " is complete!\n\n";
    exit(EXIT_SUCCESS);
}
