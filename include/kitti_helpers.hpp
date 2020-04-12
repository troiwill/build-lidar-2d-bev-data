#pragma once

#include <iomanip>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


const double kPi = 3.14159265359;

struct KittiOxts_t
{
    union
    {
        struct 
        {
            double lat, lon, alt;
            double roll, pitch, yaw;

            double vn, ve, vf, vl, vu;
            double ax, ay, az, af, al, au;
            double wx, wy, wz, wf, wl, wu;

            double posacc, velacc;
        };
        double data[25];
    };

    union
    {
        struct
        {
            int navstat, numstats, posmode, velmode, orimode;
        };
        int info[5];
    };

    KittiOxts_t(const std::vector<std::string>& oxtsData)
    {
        std::size_t i = 0;
        for (; i < 25; i++)
            data[i] = std::stod(oxtsData[i]);
        
        for (; i < 30; i++)
            info[i - 25] = std::stoi(oxtsData[i]);
    }
};

struct KittiSequence_t
{
    KittiSequence_t(const std::string& kId, const std::string& kName, const std::uint32_t kStart,
        const std::uint32_t kEnd, bool useSync)
      : _id(kId),
        _name(kName),
        _start(kStart),
        _end(kEnd)
    {
        if (useSync)
            _name += "_sync";
    }

    const std::string& id() const { return this->_id; }
    const std::string& name() const { return this->_name; }
    int start() const { return this->_start; }
    int end() const { return this->_end; }

    static std::string to_string(const std::uint32_t& id)
    {
        std::ostringstream ss;
        ss << std::setw(10) << std::setfill('0') << id;
        return ss.str();
    }

private:
    std::string _id, _name;
    std::uint32_t _start, _end;
};

Eigen::Matrix4f computePoseFromOxts(
    const KittiOxts_t& oxts,
    const float scale);

std::vector<boost::filesystem::path> getDirectoryContents(
    const boost::filesystem::path& kPath,
    bool findDirs,
    const std::string& fileExt="");

KittiOxts_t readOxtsData(
    const std::string& kOxtsPath);

void readVelodyneBin(
    const std::string& kBinPath,
    pcl::PointCloud<pcl::PointXYZI>& cloud);
