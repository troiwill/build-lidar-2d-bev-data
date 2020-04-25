#pragma once

#include <cstdlib>
#include <string>
#include <vector>


namespace kitti_bev
{
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
}
