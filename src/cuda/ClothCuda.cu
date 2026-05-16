#include "ClothCuda.cuh"
#include "CudaUtils.cuh"
#include "../Particle.h"
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(const Clock::time_point& start, const Clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

__constant__ double kSingleMove[15] = {
    0, 0.3, 0.51, 0.657, 0.7599,
    0.83193, 0.88235, 0.91765, 0.94235, 0.95965,
    0.97175, 0.98023, 0.98616, 0.99031, 0.99322
};

__constant__ double kDoubleMove[15] = {
    0, 0.3, 0.42, 0.468, 0.4872,
    0.4949, 0.498, 0.4992, 0.4997, 0.4999,
    0.4999, 0.5, 0.5, 0.5, 0.5
};

int gridSize(int count, int blockSize)
{
    int blocks = (count + blockSize - 1) / blockSize;
    return blocks > 0 ? blocks : 1;
}

__global__ void verletKernel(double* pos_y,
                             double* old_y,
                             const double* acceleration_y,
                             const unsigned char* movable,
                             double time_step2,
                             int count)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= count || !movable[i]) {
        return;
    }

    double temp = pos_y[i];
    pos_y[i] = pos_y[i] + (pos_y[i] - old_y[i]) * (1.0 - DAMPING) + acceleration_y[i] * time_step2;
    old_y[i] = temp;
}

__global__ void resetConstraintKernel(const double* pos_y,
                                      double* pos_y_snapshot,
                                      double* delta_y,
                                      int count)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= count) {
        return;
    }
    pos_y_snapshot[i] = pos_y[i];
    delta_y[i] = 0.0;
}

__global__ void constraintKernel(const double* pos_y_snapshot,
                                 const unsigned char* movable,
                                 const int* neighbor_begin,
                                 const int* neighbor_indices,
                                 double* delta_y,
                                 int count,
                                 int constraint_iterations)
{
    int p1 = blockIdx.x * blockDim.x + threadIdx.x;
    if (p1 >= count || !movable[p1]) {
        return;
    }

    double localDelta = 0.0;
    int begin = neighbor_begin[p1];
    int end = neighbor_begin[p1 + 1];

    for (int offset = begin; offset < end; offset++) {
        int p2 = neighbor_indices[offset];
        double correction = pos_y_snapshot[p2] - pos_y_snapshot[p1];
        if (movable[p2]) {
            double factor = constraint_iterations > 14 ? 0.5 : kDoubleMove[constraint_iterations];
            localDelta += correction * factor;
        } else {
            double factor = constraint_iterations > 14 ? 1.0 : kSingleMove[constraint_iterations];
            localDelta += correction * factor;
        }
    }

    delta_y[p1] = localDelta;
}

__global__ void applyDeltaKernel(double* pos_y,
                                 const double* delta_y,
                                 const unsigned char* movable,
                                 int count)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count && movable[i]) {
        pos_y[i] += delta_y[i];
    }
}

__global__ void collisionKernel(double* pos_y,
                                unsigned char* movable,
                                const double* heightvals,
                                int count)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= count) {
        return;
    }

    if (pos_y[i] < heightvals[i]) {
        if (movable[i]) {
            pos_y[i] = heightvals[i];
        }
        movable[i] = 0;
    }
}

__global__ void diffKernel(const double* pos_y,
                           const double* old_y,
                           const unsigned char* movable,
                           double* diff,
                           int count)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= count) {
        return;
    }
    diff[i] = movable[i] ? fabs(old_y[i] - pos_y[i]) : 0.0;
}

__global__ void classifyKernel(const double* pc_x,
                               const double* pc_y,
                               const double* pc_z,
                               int point_count,
                               const double* pos_y,
                               double origin_x,
                               double origin_z,
                               double step_x,
                               double step_y,
                               int width,
                               int height,
                               double class_threshold,
                               unsigned char* ground_flags)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= point_count) {
        return;
    }

    double deltaX = pc_x[i] - origin_x;
    double deltaZ = pc_z[i] - origin_z;
    int col0 = static_cast<int>(deltaX / step_x);
    int row0 = static_cast<int>(deltaZ / step_y);
    int col1 = col0 + 1;
    int row1 = row0;
    int col2 = col0 + 1;
    int row2 = row0 + 1;
    int col3 = col0;
    int row3 = row0 + 1;

    if (col0 < 0 || row0 < 0 || col1 >= width || row2 >= height) {
        ground_flags[i] = 0;
        return;
    }

    double subdeltaX = (deltaX - col0 * step_x) / step_x;
    double subdeltaZ = (deltaZ - row0 * step_y) / step_y;

    double fxy = pos_y[row0 * width + col0] * (1 - subdeltaX) * (1 - subdeltaZ) +
                 pos_y[row3 * width + col3] * (1 - subdeltaX) * subdeltaZ +
                 pos_y[row2 * width + col2] * subdeltaX * subdeltaZ +
                 pos_y[row1 * width + col1] * subdeltaX * (1 - subdeltaZ);

    double height_var = fxy - pc_y[i];
    ground_flags[i] = fabs(height_var) < class_threshold ? 1 : 0;
}

template <typename T>
T* copyVectorToDevice(const std::vector<T>& source)
{
    T* device = 0;
    std::size_t bytes = source.size() * sizeof(T);
    CSF_CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&device), bytes));
    if (bytes > 0) {
        CSF_CUDA_CHECK(cudaMemcpy(device, source.data(), bytes, cudaMemcpyHostToDevice));
    }
    return device;
}

template <typename T>
void copyDeviceToVector(const T* device, std::vector<T>& target)
{
    std::size_t bytes = target.size() * sizeof(T);
    if (bytes > 0) {
        CSF_CUDA_CHECK(cudaMemcpy(target.data(), device, bytes, cudaMemcpyDeviceToHost));
    }
}

void freeIfAllocated(void* ptr)
{
    if (ptr) {
        cudaFree(ptr);
    }
}

}

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
                   int& iterations_run)
{
    CSF_CUDA_CHECK(cudaSetDevice(device_id));

    int count = cloth.getSize();
    int threads = 256;
    int blocks = gridSize(count, threads);

    double* d_pos_y = 0;
    double* d_old_y = 0;
    double* d_acceleration_y = 0;
    double* d_heightvals = 0;
    double* d_pos_y_snapshot = 0;
    double* d_delta_y = 0;
    double* d_diff = 0;
    unsigned char* d_movable = 0;
    int* d_neighbor_begin = 0;
    int* d_neighbor_indices = 0;

    Clock::time_point simulationStart = Clock::now();

    try {
        d_pos_y = copyVectorToDevice(cloth.pos_y);
        d_old_y = copyVectorToDevice(cloth.old_y);
        d_acceleration_y = copyVectorToDevice(cloth.acceleration_y);
        d_heightvals = copyVectorToDevice(cloth.heightvals);
        d_movable = copyVectorToDevice(cloth.movable);
        d_neighbor_begin = copyVectorToDevice(cloth.neighborBeginVector());
        d_neighbor_indices = copyVectorToDevice(cloth.neighborIndexVector());

        CSF_CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_pos_y_snapshot), static_cast<std::size_t>(count) * sizeof(double)));
        CSF_CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_delta_y), static_cast<std::size_t>(count) * sizeof(double)));
        CSF_CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_diff), static_cast<std::size_t>(count) * sizeof(double)));

        for (int i = 0; i < iterations; i++) {
            Clock::time_point stageStart = Clock::now();
            verletKernel<<<blocks, threads>>>(d_pos_y, d_old_y, d_acceleration_y, d_movable, time_step2, count);
            CSF_CUDA_CHECK(cudaGetLastError());
            CSF_CUDA_CHECK(cudaDeviceSynchronize());
            step_profile.verlet_ms += elapsedMs(stageStart, Clock::now());

            stageStart = Clock::now();
            resetConstraintKernel<<<blocks, threads>>>(d_pos_y, d_pos_y_snapshot, d_delta_y, count);
            CSF_CUDA_CHECK(cudaGetLastError());
            constraintKernel<<<blocks, threads>>>(
                d_pos_y_snapshot,
                d_movable,
                d_neighbor_begin,
                d_neighbor_indices,
                d_delta_y,
                count,
                constraint_iterations);
            CSF_CUDA_CHECK(cudaGetLastError());
            applyDeltaKernel<<<blocks, threads>>>(d_pos_y, d_delta_y, d_movable, count);
            CSF_CUDA_CHECK(cudaGetLastError());
            CSF_CUDA_CHECK(cudaDeviceSynchronize());
            step_profile.constraint_ms += elapsedMs(stageStart, Clock::now());

            stageStart = Clock::now();
            diffKernel<<<blocks, threads>>>(d_pos_y, d_old_y, d_movable, d_diff, count);
            CSF_CUDA_CHECK(cudaGetLastError());
            double maxDiff = reduceMaxDevice(d_diff, count);
            CSF_CUDA_CHECK(cudaDeviceSynchronize());
            step_profile.maxdiff_ms += elapsedMs(stageStart, Clock::now());

            stageStart = Clock::now();
            collisionKernel<<<blocks, threads>>>(d_pos_y, d_movable, d_heightvals, count);
            CSF_CUDA_CHECK(cudaGetLastError());
            CSF_CUDA_CHECK(cudaDeviceSynchronize());
            collision_ms += elapsedMs(stageStart, Clock::now());

            iterations_run = i + 1;
            if ((maxDiff != 0) && (maxDiff < 0.005)) {
                break;
            }
        }

        timestep_ms = step_profile.verlet_ms + step_profile.constraint_ms + step_profile.maxdiff_ms;
        simulation_ms = elapsedMs(simulationStart, Clock::now());

        copyDeviceToVector(d_pos_y, cloth.pos_y);
        copyDeviceToVector(d_old_y, cloth.old_y);
        copyDeviceToVector(d_movable, cloth.movable);
    } catch (...) {
        freeIfAllocated(d_pos_y);
        freeIfAllocated(d_old_y);
        freeIfAllocated(d_acceleration_y);
        freeIfAllocated(d_heightvals);
        freeIfAllocated(d_pos_y_snapshot);
        freeIfAllocated(d_delta_y);
        freeIfAllocated(d_diff);
        freeIfAllocated(d_movable);
        freeIfAllocated(d_neighbor_begin);
        freeIfAllocated(d_neighbor_indices);
        throw;
    }

    freeIfAllocated(d_pos_y);
    freeIfAllocated(d_old_y);
    freeIfAllocated(d_acceleration_y);
    freeIfAllocated(d_heightvals);
    freeIfAllocated(d_pos_y_snapshot);
    freeIfAllocated(d_delta_y);
    freeIfAllocated(d_diff);
    freeIfAllocated(d_movable);
    freeIfAllocated(d_neighbor_begin);
    freeIfAllocated(d_neighbor_indices);
}

void classify(const ClothSoA& cloth,
              const csf::PointCloud& pc,
              double class_threshold,
              int device_id,
              std::vector<int>& groundIndexes,
              std::vector<int>& offGroundIndexes)
{
    CSF_CUDA_CHECK(cudaSetDevice(device_id));

    int pointCount = static_cast<int>(pc.size());
    std::vector<double> pc_x(pointCount);
    std::vector<double> pc_y(pointCount);
    std::vector<double> pc_z(pointCount);
    for (int i = 0; i < pointCount; i++) {
        pc_x[i] = pc[i].x;
        pc_y[i] = pc[i].y;
        pc_z[i] = pc[i].z;
    }

    double* d_pc_x = 0;
    double* d_pc_y = 0;
    double* d_pc_z = 0;
    double* d_pos_y = 0;
    unsigned char* d_ground_flags = 0;

    try {
        d_pc_x = copyVectorToDevice(pc_x);
        d_pc_y = copyVectorToDevice(pc_y);
        d_pc_z = copyVectorToDevice(pc_z);
        d_pos_y = copyVectorToDevice(cloth.pos_y);
        CSF_CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_ground_flags), static_cast<std::size_t>(pointCount) * sizeof(unsigned char)));

        int threads = 256;
        int blocks = gridSize(pointCount, threads);
        classifyKernel<<<blocks, threads>>>(
            d_pc_x,
            d_pc_y,
            d_pc_z,
            pointCount,
            d_pos_y,
            cloth.origin_pos.f[0],
            cloth.origin_pos.f[2],
            cloth.step_x,
            cloth.step_y,
            cloth.num_particles_width,
            cloth.num_particles_height,
            class_threshold,
            d_ground_flags);
        CSF_CUDA_CHECK(cudaGetLastError());
        CSF_CUDA_CHECK(cudaDeviceSynchronize());

        std::vector<unsigned char> groundFlags(pointCount);
        CSF_CUDA_CHECK(cudaMemcpy(groundFlags.data(), d_ground_flags,
                                  static_cast<std::size_t>(pointCount) * sizeof(unsigned char),
                                  cudaMemcpyDeviceToHost));

        groundIndexes.clear();
        offGroundIndexes.clear();
        for (int i = 0; i < pointCount; i++) {
            if (groundFlags[i]) {
                groundIndexes.push_back(i);
            } else {
                offGroundIndexes.push_back(i);
            }
        }
    } catch (...) {
        freeIfAllocated(d_pc_x);
        freeIfAllocated(d_pc_y);
        freeIfAllocated(d_pc_z);
        freeIfAllocated(d_pos_y);
        freeIfAllocated(d_ground_flags);
        throw;
    }

    freeIfAllocated(d_pc_x);
    freeIfAllocated(d_pc_y);
    freeIfAllocated(d_pc_z);
    freeIfAllocated(d_pos_y);
    freeIfAllocated(d_ground_flags);
}

}
