#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>
#include <utility>

#include <Eigen/Core>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>


class PointCloudDim2D;
struct TransformXYTheta;

typedef Eigen::Matrix<std::uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RMatrixXui8;

void writeBGM(const std::string& kSavePath, const RMatrixXui8& kMat);

void writeScanInfo(const std::string& KInfofilename, const std::vector<std::string>& kVelonames,
                   const std::vector<TransformXYTheta>& kTFs, 
                   const std::pair<std::uint32_t, std::uint32_t>& kMapImgSize,
                   const float kResolution);

inline
std::array<int, 2> computePixelLoc(
    int imgHeight,
    int imgWidth,
    float x,
    float y,
    float resolution)
{
    assert(imgHeight > 0 && imgWidth > 0);
    assert(x >= 0.f && y >= 0.f);
    assert(resolution > 0.f);

    return std::array<int, 2>{
        std::min(imgHeight - static_cast<int>(floor(x / resolution)), imgHeight - 1),
        std::min( imgWidth - static_cast<int>(floor(y / resolution)),  imgWidth - 1)
    };
}

template <typename PointT>
void aggregatePointClouds(
    const std::vector<pcl::PointCloud<PointT>>& kIndivClouds,
    pcl::PointCloud<PointT>& aggCloud)
{
    for (auto it = kIndivClouds.begin(); it != kIndivClouds.end(); ++it)
        aggCloud += *it;
}

template<typename PointT>
RMatrixXui8 buildBEVFromCloud(
    const pcl::PointCloud<PointT>& kCloud,
    const float kXdiff,
    const float kYdiff,
    const float outRes,
    bool useZvals = false,
    float minZ = 0.f)
{
    // Sanity checks.
    assert(kXdiff > 0.f && kYdiff > 0.f && outRes > 0.f);

    // Compute the image size.
    auto imgHeight = static_cast<int>(ceil(kXdiff / outRes)) + 1;
    auto imgWidth = static_cast<int>(ceil(kYdiff / outRes)) + 1;
    
    // Create the bird's eye matrix (image) and iteratively fill the matrix.
    RMatrixXui8 bev = RMatrixXui8::Zero(imgHeight, imgWidth);
    for (std::size_t i = 0; i < kCloud.size(); i++)
    {
        const PointT& pt = kCloud[i];
        float pixelValue = 0.0f;
        // max / min from zlen
        if (useZvals)
        {
            //pixelValue = (pt.z - kMinPt.z) / (kMaxPt.z - kMinPt.z);
            pixelValue = (pt.z + minZ) / (minZ * 2);
        }
        else
        {
            pixelValue = pt.intensity;
        }
        pixelValue = std::min(std::max(pixelValue * 255.0f, 0.0f), 255.0f);

        auto pxLoc = computePixelLoc(imgHeight, imgWidth, pt.x, pt.y, outRes);
        auto row = pxLoc[0];
        auto col = pxLoc[1];

        bev(row, col) = std::max(static_cast<std::uint8_t>(pixelValue), bev(row, col));
    }
    return bev;
}

template<typename PointT>
void extractPointCloudROI(
    const pcl::PointCloud<PointT>& kOriginalPC,
    const float kXlen,
    const float kYlen,
    pcl::PointCloud<PointT>& extractedPC)
{
    // Sanity checks.
    assert(kOriginalPC.size() > 0);
    assert(kXlen > 0.f && kYlen > 0.f);

    float x = 0.f, y = 0.f;
    for (auto pt_it = kOriginalPC.points.begin(); pt_it != kOriginalPC.points.end(); ++pt_it)
    {
        x = pt_it->x;
        y = pt_it->y;
        if (-kXlen < x && x < kXlen && -kYlen < y && y < kYlen)
            extractedPC.push_back(*pt_it);
    }
}

template<typename PointT>
void extractPointCloudROI(
    const pcl::PointCloud<PointT>& kOriginalPC,
    const float kRadius,
    pcl::PointCloud<PointT>& extractedPC)
{
    // Sanity checks.
    assert(kOriginalPC.size() > 0);
    assert(kRadius > 0.f);

    float d = 0.f;
    for (auto pt_it = kOriginalPC.points.begin(); pt_it != kOriginalPC.points.end(); ++pt_it)
    {
        d = std::sqrt(std::pow(pt_it->x, 2) + std::pow(pt_it->y, 2));
        if (d < kRadius)
            extractedPC.push_back(*pt_it);
    }
}

template<typename PointT>
void translateDataXY(
    pcl::PointCloud<PointT>& cloud,
    const Eigen::Vector2f& kShift)
{
    // Sanity check.
    assert(cloud.size() > 0);

    const float dx = -kShift[0], dy = -kShift[1];
    for (auto it = cloud.points.begin(); it != cloud.points.end(); ++it)
    {
        it->x += dx;
        it->y += dy;
    }
}

template<typename T>
void translateDataXY(
    std::vector<T>& data,
    const Eigen::Vector2f& kShift)
{
    // Sanity check.
    assert(data.size() > 0);

    const float dx = -kShift[0], dy = -kShift[1];
    for (auto it = data.begin(); it != data.end(); ++it)
    {
        it->x += dx;
        it->y += dy;
    }
}
