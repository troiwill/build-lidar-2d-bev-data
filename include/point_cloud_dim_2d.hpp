#pragma once

#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>


namespace bev2d
{
    class PointCloudDim2D
    {
    public:
        template<typename PointT>
        static PointCloudDim2D getMinMaxData2D(const pcl::PointCloud<PointT>& kCloud)
        {
            PointT minPt, maxPt;
            pcl::getMinMax3D(kCloud, minPt, maxPt);
            
            return PointCloudDim2D(
                Eigen::Vector2f(minPt.x, minPt.y),
                Eigen::Vector2f(maxPt.x, maxPt.y)
            );
        }

        float getCloudLenX() const
        {
            return maxPt.x() - minPt.x();
        }

        float getCloudLenY() const
        {
            return maxPt.y() - minPt.y();
        }

        const Eigen::Vector2f& getCloudMaxPt() const
        {
            return maxPt;
        }

        const Eigen::Vector2f& getCloudMinPt() const
        {
            return minPt;
        }

    private:
        PointCloudDim2D(const Eigen::Vector2f& _minPt, const Eigen::Vector2f& _maxPt)
        : minPt(_minPt), maxPt(_maxPt)
        { }

        Eigen::Vector2f minPt, maxPt;
    };
}
