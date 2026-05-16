#ifndef _C2CDIST_SOA_H_
#define _C2CDIST_SOA_H_

#include "ClothSoA.h"
#include "point_cloud.h"
#include <vector>

class c2cdistSoA {
public:
    c2cdistSoA(double threshold) : class_threshold(threshold) {}

    void calCloud2CloudDist(ClothSoA& cloth,
                            csf::PointCloud& pc,
                            std::vector<int>& groundIndexes,
                            std::vector<int>& offGroundIndexes);

private:
    double class_threshold;
};

#endif
