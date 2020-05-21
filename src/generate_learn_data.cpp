#include "kitti_helpers.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <queue>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <Eigen/Core>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace boostfs = boost::filesystem;
namespace boostpo = boost::program_options;

using namespace Eigen;
using namespace kitti_bev;
using namespace pcl;
using namespace std;

mutex thdmtx;

struct GeneratorParams
{
public:
    GeneratorParams(const GeneratorParams& kParam)
      : _seqId(kParam._seqId),
        _scanId(kParam._scanId),
        _bstrapId(kParam._bstrapId),
        _dx(kParam._dx),
        _dy(kParam._dy),
        _dt(kParam._dt),
        _centerPt(kParam._centerPt)
    { }

    GeneratorParams(const string& kSeqId, const string& kScanId, const int kBstrapId,
        const Vector2f& kCenterPt, const float kDx, const float kDy, const float kDt)
      : _seqId(kSeqId),
        _scanId(kScanId),
        _bstrapId(),
        _centerPt(kCenterPt),
        _dx(kDx),
        _dy(kDy),
        _dt(kDt)
    {
        string bstrapStr(to_string(kBstrapId));
        _bstrapId = string(2 - bstrapStr.length(), '0') + bstrapStr;
    }

    const string& seqId() const { return _seqId; }
    const string& scanId() const { return _scanId; }
    const string& bstrapId() const { return _bstrapId; }

    const Vector2f& center() const { return _centerPt; }

    float dx() const { return _dx; }
    float dy() const { return _dy; }
    float dt() const { return _dt; }

    string getDataId() const 
    {
        return this->_seqId + "_" + this->_scanId + "_" + this->_bstrapId;
    }

    static string getHeader()
    {
        return "dataId,wx,wy,dx,dy,dt";
    }

    friend ostream& operator<<(ostream& out, const GeneratorParams& pm)
    {
        out << pm.getDataId() << "," << pm.center().x() << "," << pm.center().y()
            << "," << pm.dx() << "," << pm.dy() << "," << pm.dt();
        return out;
    }

private:
    string _seqId, _scanId, _bstrapId;
    float _dx, _dy, _dt;
    Vector2f _centerPt;
};

struct ScanAttrs
{
    ScanAttrs(const string& kAttrsStr)
      : scanId(),
        center()
    {
        vector<string> attrs;
        boost::split(attrs, kAttrsStr, boost::is_any_of(","));
        this->scanId = attrs[0];
        this->center = Vector2f{ stof(attrs[1]), stof(attrs[2]) };
    }

    string scanId;
    Vector2f center;
};

void createBEVimage(const PointCloud<PointXYZI>& kCloud,
    const vector<int>& kCloudIdx, const Vector2f& kCenterPt, const float kScanLen,
    const int kNPixels, const float kBevRes, vector<uint8_t>& bevImg)
{
    // Sanity checks.
    int x_idx = 0, y_idx = 0, bev_idx = 0;
    float x_min = kCenterPt.x() - (kScanLen / 2);
    float y_min = kCenterPt.y() - (kScanLen / 2);
    const uint8_t kMaxPixelVal = 255, kOne = 1;
    for (const int idx: kCloudIdx)
    {
        const PointXYZI& kPt = kCloud[idx];
        x_idx = std::min(kNPixels - static_cast<int>((kPt.x - x_min) / kBevRes), kNPixels - 1);
        y_idx = std::min(kNPixels - static_cast<int>((kPt.y - y_min) / kBevRes), kNPixels - 1);

        bev_idx = (y_idx * kNPixels) + x_idx;
        if (bevImg[bev_idx] < kMaxPixelVal)
            bevImg[bev_idx] += kOne;
    }
}

void writeBEVimage(const string& kBevFilepath, const int kNPixels, const vector<uint8_t>& kBevImg)
{
    // Open the file.
    fstream bevfile(kBevFilepath, ios::out | ios::binary);
    if (!bevfile.good())
    {
        cerr << "Cannot open or create PCD binary file: " << kBevFilepath << endl;
        exit(EXIT_FAILURE);
    }
    bevfile << "P5\n" << kNPixels << " " << kNPixels << "\n255\n";
    for (const uint8_t& kVal : kBevImg)
        bevfile << kVal;
    bevfile.close();
}

void filterPointsXY(const PointCloud<PointXYZI>::Ptr& kCloud, const Vector2f& kMinPt,
    const Vector2f& kMaxPt, vector<int>& filteredPoints)
{
    // Create the PassThrough filter to get the points of interest.
    PassThrough<PointXYZI> ptfilter;
    ptfilter.setInputCloud(kCloud);

    // Filter along the x-axis.
    vector<int> x_indices;
    ptfilter.setFilterFieldName("x");
    ptfilter.setFilterLimits(kMinPt.x(), kMaxPt.x());
    ptfilter.filter(x_indices);

    // Filter along the y-axis and place the indices in the output vector.
    IndicesPtr xptr(new vector<int>(x_indices));
    // for (const int& xval : x_indices) xptr->push_back(xval);
    ptfilter.setIndices(xptr);
    ptfilter.setFilterFieldName("y");
    ptfilter.setFilterLimits(kMinPt.y(), kMaxPt.y());
    ptfilter.filter(filteredPoints);
}

void generateBevSample(const octree::OctreePointCloudSearch<PointXYZI>& kSearchTree,
    queue<GeneratorParams>& paramQ, int& nDone, const int kQueueLen, 
    const boostfs::path& kSeqPath, const boostfs::path& kSavePath, const float kBevRes,
    const float kScanLen)
{
    // Define the BEV image buffer.
    const int kNPixels = static_cast<int>(kScanLen / kBevRes);
    vector<uint8_t> scanBev(kNPixels * kNPixels, 0);
    vector<uint8_t>  mapBev(kNPixels * kNPixels, 0);

    // Define the point cloud pointers.
    PointCloud<PointXYZI>::ConstPtr kMapPtr(kSearchTree.getInputCloud());
    PointCloud<PointXYZI>::Ptr scan(new PointCloud<PointXYZI>);
    PointCloud<PointXYZI>::Ptr tfdScan(new PointCloud<PointXYZI>);
    
    // Define the translation and rotation matrices for the LiDAR scan.
    Matrix4f transInvMat(Matrix4f::Identity());
    Matrix4f rotMat(Matrix4f::Identity());

    // Define the search box min and max points.
    Vector3f searchBoxMinPt{0.0f, 0.0f, -1000.0f};
    Vector3f searchBoxMaxPt{0.0f, 0.0f,  1000.0f};
    
    vector<int> scanIndices;
    vector<int> mapCutOutIndices;

    const float kHalfScanLen = kScanLen / 2.0f;
    const Vector2f kZeroOrigin{0.0f, 0.0f};
    Vector2f newCenterPt;

    string scanBevName, mapBevName;
    while (true)
    {
        // Get the next item from the queue.
        thdmtx.lock();
        if (paramQ.empty()) // Exit the loop?
        {
            thdmtx.unlock();
            break;
        }
        const GeneratorParams kParam(paramQ.front());
        paramQ.pop();
        ++nDone;
        printf("Progress: % 7d / % 7d (% 6.2f%%)\r", nDone, kQueueLen,
            (static_cast<float>(nDone) * 100.0f / kQueueLen));
        cout << flush;
        thdmtx.unlock();

        const Vector2f& kCenterPt(kParam.center());

        // Load the pointcloud data from disk.
        scan->clear();
        readVelodyneBin((kSeqPath / kParam.scanId()).string() + ".bin", *scan);

        // Shift points to origin and rotate points about the z-axis by dt radians.
        tfdScan->clear();
        transInvMat.block<2,1>(0,3) = -1.0 * kCenterPt;
        rotMat.block<3,3>(0,0) = AngleAxisf(kParam.dt(), Vector3f::UnitZ()).matrix();
        transformPointCloud(*scan, *tfdScan, rotMat * transInvMat);

        // Use PassThrough filter to get points for the transformed scan.
        scanIndices.clear();
        filterPointsXY(tfdScan, Vector2f{ -kHalfScanLen, -kHalfScanLen },
            Vector2f{ kHalfScanLen, kHalfScanLen }, scanIndices);
        if (scanIndices.size() == 0)
            throw "Cannot find points using the pass through filter!";
        
        // Write a BEV image for the transformed LiDAR scan.
        createBEVimage(*tfdScan, scanIndices, kZeroOrigin, kScanLen, kNPixels, kBevRes, scanBev);
        scanBevName = string("scan_") + kParam.getDataId() + string(".png");
        writeBEVimage((kSavePath / scanBevName).string(), kNPixels, scanBev);

        // Get all the map points within range | p - (c + dxy) | < scan_len.
        mapCutOutIndices.clear();
        searchBoxMinPt.x() = searchBoxMaxPt.x() = newCenterPt.x() = kCenterPt.x() + kParam.dx();
        searchBoxMinPt.x() -= kHalfScanLen;
        searchBoxMaxPt.x() += kHalfScanLen;

        searchBoxMinPt.y() = searchBoxMaxPt.y() = newCenterPt.y() = kCenterPt.y() + kParam.dy();
        searchBoxMinPt.y() -= kHalfScanLen;
        searchBoxMaxPt.y() += kHalfScanLen;

        int npts = kSearchTree.boxSearch(searchBoxMinPt, searchBoxMaxPt, mapCutOutIndices);
        if (npts == 0)
            throw "Cannot find points within search box using the Octree.";
        
        // Write a BEV image for the map cut out.
        createBEVimage(*kMapPtr, mapCutOutIndices, newCenterPt, kScanLen, kNPixels, kBevRes,
            mapBev);
        mapBevName = string("map_") + kParam.getDataId() + string(".png");
        writeBEVimage((kSavePath / mapBevName).string(), kNPixels, mapBev);

        // Clear the BEV buffers.
        for (size_t i = 0; i < scanBev.size(); i++)
            scanBev[i] = mapBev[i] = 0;
    }
}

void readfile(const string& kFilePath, vector<string>& lines)
{
    // Open the file.
    ifstream inf(kFilePath, ifstream::in);
    string fline;

    // Read the lines from the file.
    while (getline(inf, fline))
    {
        if (fline.empty() || fline.find("scanId") != string::npos)
            continue;
        lines.push_back(fline);
    }
    
    // Close the file.
    inf.close();
}

int main(int argc, char** argv)
{
    int nsamplesPerScan = 0, scanLen = 0;;
    float transMax = 0.0f, rotMax = 0.0f, bevRes = 0.0f;
    size_t nworkers = 0;
    string setfilepath, seqroot, saveroot;

    boostpo::options_description desc("Build Kitti 2D BEV Data Program Options");
    desc.add_options()
        ("help", "Print help message.")
        ("seqroot", boostpo::value<string>(&seqroot)->required(), "Kitti base directory.")
        ("saveroot", boostpo::value<string>(&saveroot)->required(), "Output directory for Kitti learning data.")
        ("setfile", boostpo::value<string>(&setfilepath)->required(), "File that lists the sequences to use.")
        ("bevres", boostpo::value<float>(&bevRes)->required(), "BEV resolution (meters / pixel).")
        ("sps", boostpo::value<int>(&nsamplesPerScan)->required(), "Number of samples to generate per scan.")
        ("scanlen", boostpo::value<int>(&scanLen)->required(), "Total scan length (in meters).")
        ("transmax", boostpo::value<float>(&transMax)->required(), "Max translation (in meters).")
        ("rotmax", boostpo::value<float>(&rotMax)->required(), "Max rotation (in degrees).")
        ("nworkers", boostpo::value<size_t>(&nworkers)->required(), "Number of workers to create data.")
        ;

    boostpo::variables_map vm;
    boostpo::store(boostpo::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help"))
    {
        cout << desc << endl;
        exit(EXIT_SUCCESS);
    }
    boostpo::notify(vm);

    // Create the paths for sequence and save.
    boostfs::path seqRootPath(seqroot);
    boostfs::path savePath(saveroot);
    float rotMaxDegs = rotMax;
    rotMax = rotMaxDegs / 180.0f * M_PI;
    
    cout << "\nData Generation Parameters:";
    cout << "\nSequence root dir:       " << seqroot;
    cout << "\nSave root dir:           " << saveroot;
    cout << "\nSet file:                " << setfilepath;
    cout << "\nBEV resolution (m / px): " << bevRes;
    cout << "\nSample per scan:         " << nsamplesPerScan;
    cout << "\nScan length (meters):    " << scanLen;
    cout << "\nTranslate Max (meters):  " << transMax;
    cout << "\nRotation Max (radians):  " << rotMax << " [ degrees = " << rotMaxDegs << " ].";
    cout << "\n# of workers:            " << nworkers;
    cout << endl << endl;

    // Sanity checks.
    assert(boostfs::exists(seqRootPath));
    assert(boostfs::exists(savePath));
    assert(boostfs::exists(boostfs::path(setfilepath)));
    assert(nsamplesPerScan >= 1);
    assert(bevRes > 0.0f);
    assert(scanLen >= 2);
    assert(transMax >= 0.0f);
    assert(2.0f * M_PI > rotMax && rotMax >= 0.0f);
    assert(nworkers >= 1);
    
    // Iterate over each sequence.
    cout << "Reading the set file: " << setfilepath << "...";
    vector<string> seqLines;
    readfile(setfilepath, seqLines);
    cout << " Read " << seqLines.size() << " sequences.\n\n";

    PointCloud<PointXYZI>::Ptr mapcloud(new PointCloud<PointXYZI>);
    bool isFirstWrite = true;
    for (const string& seqId : seqLines)
    {
        cout << "Processing sequence " << seqId << endl;
        boostfs::path seqpath(seqRootPath / seqId);

        // Read the map point cloud from disk.
        string mapbinfilepath((seqpath / "map.bin").string());
        cout << "Reading map pointcloud (" << mapbinfilepath << ") ... ";
        mapcloud->clear();
        readVelodyneBin(mapbinfilepath, *mapcloud);
        cout << "Done!\nMap contains " << mapcloud->points.size() << " points.\n\n";

        // Create the generator parameters for the sample function.
        unsigned randSeed = chrono::system_clock::now().time_since_epoch().count();
        default_random_engine generator;
        uniform_real_distribution<float> uniTrans(-transMax, transMax);
        uniform_real_distribution<float> uniRot(-rotMax, rotMax);

        // Read info file.
        string scanfilepath((seqpath / "info.txt").string());
        cout << "Reading information file (" << scanfilepath << ") ... ";
        vector<string> scanLines;
        readfile(scanfilepath, scanLines);
        cout << "Done!\nRead " << scanLines.size() << " entries.\n\n";

        // Create the generation sample parameters using the scan file.
        float progress = 0.0f;
        float dataSampleSize = static_cast<float>(scanLines.size() * nsamplesPerScan);
        
        string paramsfilepath((savePath / "dataparams.csv").string());
        ofstream paramfile(paramsfilepath, isFirstWrite ? ios_base::out : ios_base::app);
        
        cout << "Generating sample generating parameters.\n";
        cout << "Writing generation parameters to disk: " << paramsfilepath << endl << flush;
        queue<GeneratorParams> paramQ;
        for (const string& scline : scanLines)
        {
            ScanAttrs attr(scline);
            for (size_t bsIdx = 0; bsIdx < nsamplesPerScan; bsIdx++)
            {
                paramQ.push(GeneratorParams(seqId, attr.scanId, bsIdx, attr.center,
                    uniTrans(generator), uniTrans(generator), uniRot(generator)));

                if (isFirstWrite)
                {
                    isFirstWrite = false;
                    paramfile << GeneratorParams::getHeader() << endl;
                }
                paramfile << paramQ.back() << endl;
                
                printf("Progress: % 5.2f%% samples created.\r",
                    (++progress * 100.0f / dataSampleSize));
                cout << flush;
            }
        }
        paramfile.close();
        cout << "\nGeneration process complete!\n";
        
        // Create an Octree pointcloud.
        cout << "\nCreating Octree using pointcloud for searching..." << flush;
        octree::OctreePointCloudSearch<PointXYZI> searchTree(10.0f);
        searchTree.setInputCloud(mapcloud);
        searchTree.addPointsFromInputCloud();
        cout << "Done!\n" << flush;

        // Create and launch the threads.
        int nDone = 0;
        const int kQueueLen = paramQ.size();
        cout << "\nCreating " << nworkers << " threads to generate data.\n" << flush;
        thread* thds = new thread[nworkers];
        for (size_t thdIdx = 0; thdIdx < nworkers; thdIdx++)
        {
            thds[thdIdx] = thread(generateBevSample, ref(searchTree), ref(paramQ), ref(nDone),
                kQueueLen, ref(seqpath), ref(savePath), bevRes, static_cast<float>(scanLen));
        }

        for (size_t thdi = 0; thdi < nworkers; thdi++) thds[thdi].join();
        cout << "\nGeneration completed!\n" << flush;
    }
    exit(EXIT_SUCCESS);
}
