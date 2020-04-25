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
    // Read in the OXTS data for the first scan in the sequence.
    cout << "\nLoading the first scan in sequence " << kSeq.id() << endl << flush;
    string firstScanName(KittiSequence_t::to_string(kSeq.start()) + ".txt");
    KittiOxts_t oxts = readOxtsData((kDrivepath / "oxts" / "data" / firstScanName).string());
    const float kScale = cos(oxts.lat * kPi / 180.0f);
    const Eigen::Vector3f kOrigin = computePoseFromOxts(oxts, kScale).block<3,1>(0,3);

    // Launch threads to load the Velodyne data into memory.
    size_t kNumScans = kSeq.getNumScans();
    data.reserve(kNumScans);

    for (size_t veloId = kSeq.start(), i = 1; veloId <= kSeq.end(); veloId++, i++)
    {
        string veloname = KittiSequence_t::to_string(veloId);    
        printf("Loading velodyne data... (name: %s) - Progress -> %6.2f%\r", veloname.c_str(),
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
        Eigen::Matrix4f toWorldTF = computePoseFromOxts(oxts, kScale);
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

void aggregateVelodyneData(const vector<VelodyneData_t>& kVelodata, const size_t kNumScansToAgg,
    const KittiSequence_t& kSeq, const Vector2f& mapMinCoords, vector<VelodyneData_t>& aggVeloData)
{
    // Use threads to aggregate N consecutive point clouds for "each" scan.
    if (kNumScansToAgg > 1)
    {
	cout << "Aggregating velodyne data to create point clouds. Each cloud contains "
             << kNumScansToAgg << " scans.\n" << flush;
        size_t kAggcap = kSeq.getNumScans() - kNumScansToAgg + 1;
        aggVeloData.reserve(kAggcap);
        for (size_t i = 0, j = kNumScansToAgg - 1; i < kAggcap; i++, j++)
        {
            const VelodyneData_t& ind = kVelodata[j];
            VelodyneData_t outd;
            
            printf("Aggregating velodyne data ... (name: %s) - Progress -> %6.2f%\r",
                ind.name().c_str(), static_cast<float>(100.0 * i / kAggcap));
	    cout << flush;

            for (size_t offset = 0; offset < kNumScansToAgg; offset++)
                *(outd.scan()) += *(kVelodata[j - offset].scan());
            
            outd.name() = ind.name();
            outd.toWorldTf() = ind.toWorldTf();
            outd.xytheta() = ind.xytheta();

	    aggVeloData.push_back(outd);
        }
    }
    else
    {
        cout << "Moving the data." << flush;
        aggVeloData = move(kVelodata);
    }
    cout << "\nDone!\n\nShifting scan coords using map min coords: (x = " << mapMinCoords[0]
         << ", y = " << mapMinCoords[1] << ")..." << flush;
    for (auto data_it = aggVeloData.begin(); data_it != aggVeloData.end(); ++data_it)
    {
        data_it->xytheta().x -= mapMinCoords[0];
        data_it->xytheta().y -= mapMinCoords[1];
    }
    cout << "Done!\n";
}

void generateSequenceBevMap(const vector<VelodyneData_t>& kData, const float kRes,
    const bool useInten, const boostfs::path& seqSavePath, Vector2f& mapMinCoords,
    array<uint32_t, 2>& imgSize)
{
    cout << "Aggregating point clouds to create map.\n" << flush;
    size_t vdi = 1;
    PointCloud<PointXYZI> map;
    map.points.reserve(kData.size());
    for (auto vd = kData.cbegin(); vd != kData.cend(); ++vd, ++vdi)
    {
        map += *(vd->scan());
        printf("Building drive map ... Progress -> %6.2f%\r",
            static_cast<float>(100.0 * vdi / nData));
	cout << flush;
    }
    cout << "\nAggregation complete!\n" << flush;

    // Shift the aggregated point cloud such that the minimum (x,y) is (0,0).
    cout << "Shifting cloud and origin data..." << flush;
    PointCloudDim2D cloud2dInfo = PointCloudDim2D::getMinMaxData2D(map);
    mapMinCoords = cloud2dInfo.getCloudMinPt();
    translateDataXY(map, mapMinCoords);
    
    cout << "Done!\nSaving 2D BEV LiDAR-based map..." << flush;
    auto mapbev = buildBEVFromCloud(map, cloud2dInfo.getCloudLenX(),
        cloud2dInfo.getCloudLenY(), kRes, useInten);
    writeBGM((seqSavePath / "map.pgm").string(), mapbev);
    cout << "Done!\nSave path: " << seqSavePath.string() << endl << endl << flush;

    // Set the image size (height, width).
    imgSize[0] = mapbev.rows();
    imgSize[1] = mapbev.cols();
}

int main(int argc, char** argv)
{
    // Parse the command line arguments.
    string basedir, savedir;
    float res = 0.f;
    int seqid = 0, numScansToAgg = 0, numThreads = 1;
    bool useInten = false;

    boostpo::options_description desc("Build Kitti 2D BEV Data Program Options");
    desc.add_options()
        ("help", "Print help message.")
        ("basedir", boostpo::value<string>(&basedir)->required(), "Kitti base directory.")
        ("savedir", boostpo::value<string>(&savedir)->required(), "Output directory for Kitti learning data.")
        ("seqid", boostpo::value<int>(&seqid)->required(), "The Kitti sequence ID (e.g., 09).")
        ("nscans", boostpo::value<int>(&numScansToAgg)->required(), "Number of scans to aggregate.")
        ("res", boostpo::value<float>(&res)->required(), "The resolution of BEV images (in meters).")
        ("useInten", boostpo::bool_switch(&useInten), "Use intensity values from LiDAR.")
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
    cout << "* LiDAR BEV resolution:   " << res << " meters\n";
    numScansToAgg = max(numScansToAgg, 1);
    cout << "* Num scans to aggregate: " << numScansToAgg << endl;
    if (useInten)
        cout << "* Using LiDAR intensity values.\n";

    cout << "\n\n";
    
    // Extract the appropriate Kitti sequence.
    const KittiSequence_t& kSeq = KittiSequence_t::getSeqInfo(seqid);
    cout << "Processing sequence: " << kSeq.id() << " (raw data = " << kSeq.name() << ")\n";
    cout << "Sequence should have " << kSeq.getNumScans() << " scans.\n";

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
    Vector2f mapMinCoords;
    array<uint32_t,2> imgSize;
    generateSequenceBevMap(velodata, res, useInten, seqSavePath, mapMinCoords, imgSize);

    // Aggregate the Velodyne data if necessary.
    vector<VelodyneData_t> aggVeloData;
    aggregateVelodyneData(velodata, numScansToAgg, kSeq, mapMinCoords, aggVeloData);
    velodata.clear();

    // Write the (aggregated) Velodyne data to disk.
    writeVelodyneData(seqSavePath, aggVeloData, imgSize[0], imgSize[1], res);

    exit(EXIT_SUCCESS);
}
