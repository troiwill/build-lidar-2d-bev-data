#pragma once

struct TransformXYTheta
{
    TransformXYTheta(float _x = 0.f, float _y = 0.f, float _yaw = 0.f)
      : x(_x),
        y(_y),
        yaw(_yaw)
    { }

    union
    {
        struct
        {
            float x, y, yaw;
        };
        float data[3];
    };
};
