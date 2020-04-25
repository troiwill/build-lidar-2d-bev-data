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

namespace boostfs = boost::filesystem;

using namespace std;


namespace bev2d
{
    struct VelodyneData_t;

    typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RMatrixXui8;

    void writeBGM(const string& kSavePath, const RMatrixXui8& kMat);

    void writePCDbin(const string& kPcdSavePath, const pcl::PointCloud<pcl::PointXYZI>& kCloud);

    void writeVelodyneData(const boostfs::path& kSavePath, const vector<VelodyneData_t>& kData,
        const uint32_t kMapImgHeight, const uint32_t kMapImgWidth, const float kRes);

    inline
    array<uint32_t, 2> computePixelLoc(uint32_t imgHeight, uint32_t imgWidth, float x, float y, 
        float res)
    {
        assert(imgHeight > 0 && imgWidth > 0);
        assert(x >= 0.f && y >= 0.f);
        assert(res > 0.f);

        return std::array<uint32_t, 2>{
            std::min(imgHeight - static_cast<uint32_t>(floor(x / res)), imgHeight - 1),
            std::min( imgWidth - static_cast<uint32_t>(floor(y / res)),  imgWidth - 1)
        };
    }

    template<typename T>
    inline T eucddist(const T& x0, const T& y0, const T& x1, const T& y1)
    {
        return std::sqrt(std::pow(x0 - x1, 2) + std::pow(y0 - y1, 2));
    }

    template<typename PointT>
    RMatrixXui8 buildBEVFromCloud(
        const pcl::PointCloud<PointT>& kCloud,
        const float kXdiff,
        const float kYdiff,
        const float outRes,
        bool useIntensity)
    {
        // Sanity checks.
        assert(kXdiff > 0.f && kYdiff > 0.f && outRes > 0.f);

        // Compute the image size.
        auto imgHeight = static_cast<int>(ceil(kXdiff / outRes));
        auto imgWidth = static_cast<int>(ceil(kYdiff / outRes));
        
        // Create the bird's eye matrix (image) and iteratively fill the matrix.
        RMatrixXui8 bev = RMatrixXui8::Zero(imgHeight, imgWidth);
        for (std::size_t i = 0; i < kCloud.size(); i++)
        {
            const PointT& pt = kCloud[i];
            std::uint8_t newPixelValue = 0;

            auto pxLoc = computePixelLoc(imgHeight, imgWidth, pt.x, pt.y, outRes);
            auto row = pxLoc[0];
            auto col = pxLoc[1];
            std::uint8_t oldPixelValue = bev(row, col);

            if (useIntensity)
            {
                newPixelValue = static_cast<std::uint8_t>(std::min(std::max(
                    pt.intensity * 255.0f, 0.0f), 255.0f));
            }
            else
            {
                newPixelValue = oldPixelValue;
                if (oldPixelValue < 255)
                    newPixelValue += 1;
            }
            bev(row, col) = newPixelValue;
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
}
