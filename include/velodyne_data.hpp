#pragma once

#include "transform_xyt.hpp"

#include <string>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace Eigen;
using namespace pcl;
using namespace std;


namespace bev2d
{
    struct VelodyneData_t
    {
        VelodyneData_t()
          : _name(), _scan(new PointCloud<PointXYZI>()), _toWorldTf(), _xytheta()
        { }

        string& name() { return _name; }
        const string& name() const { return _name; }

        PointCloud<PointXYZI>::Ptr& scan() { return _scan; }
        const PointCloud<PointXYZI>::Ptr& scan() const { return _scan; }

        Matrix4f& toWorldTf() { return _toWorldTf; }
        const Matrix4f& toWorldTf() const { return _toWorldTf; }

        TransformXYTheta& xytheta() { return _xytheta; }
        const TransformXYTheta& xytheta() const { return _xytheta; }

    private:
        string _name;
        PointCloud<PointXYZI>::Ptr _scan;
        Matrix4f _toWorldTf;
        TransformXYTheta _xytheta;
    };
}
