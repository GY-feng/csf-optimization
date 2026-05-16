#ifndef _CSF_CLOTH_CUDA_CUH_
#define _CSF_CLOTH_CUDA_CUH_

#include "../ClothSoA.h"
#include "../point_cloud.h"
#include <vector>

namespace csf_cuda {

void runSimulation(ClothSoA& cloth,
                   int iterations,
                   int constraint_iterations,
                   double time_step2,
                   int device_id,
                   ClothStepProfile& step_profile,
                   double& timestep_ms,
                   double& collision_ms,
                   double& simulation_ms,
                   int& iterations_run);

void classify(const ClothSoA& cloth,
              const csf::PointCloud& pc,
              double class_threshold,
              int device_id,
              std::vector<int>& groundIndexes,
              std::vector<int>& offGroundIndexes);

}

#endif
