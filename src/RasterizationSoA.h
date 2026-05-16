#ifndef _RASTERIZATION_SOA_H_
#define _RASTERIZATION_SOA_H_

#include "ClothSoA.h"
#include "point_cloud.h"

#define SQUARE_DIST_SOA(x1, y1, x2, y2) \
    (((x1) - (x2)) * ((x1) - (x2)) + ((y1) - (y2)) * ((y1) - (y2)))

class RasterizationSoA {
public:
    static double findHeightValByNeighbor(int index, ClothSoA& cloth);
    static double findHeightValByScanline(int index, ClothSoA& cloth);
    static void RasterTerrain(ClothSoA& cloth, csf::PointCloud& pc);
};

#endif
