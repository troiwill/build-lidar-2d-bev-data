#pragma once


namespace bev2d
{
    struct TransformXYTheta
    {
        TransformXYTheta(float _x = 0.f, float _y = 0.f, float _yaw = 0.f)
          : x(_x),
            y(_y),
            yaw(_yaw)
        { }

        float x;
        float y;
        float yaw;
    };
}
