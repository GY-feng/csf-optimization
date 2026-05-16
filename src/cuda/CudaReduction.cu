#include "CudaUtils.cuh"

namespace {

__global__ void reduceMaxKernel(const double* values,
                                double* partial,
                                int count)
{
    extern __shared__ double cache[];
    int tid = threadIdx.x;
    int global = blockIdx.x * blockDim.x + threadIdx.x;

    double localMax = 0.0;
    for (int i = global; i < count; i += blockDim.x * gridDim.x) {
        double value = values[i];
        if (value > localMax) {
            localMax = value;
        }
    }

    cache[tid] = localMax;
    __syncthreads();

    for (int stride = blockDim.x / 2; stride > 0; stride >>= 1) {
        if (tid < stride && cache[tid + stride] > cache[tid]) {
            cache[tid] = cache[tid + stride];
        }
        __syncthreads();
    }

    if (tid == 0) {
        partial[blockIdx.x] = cache[0];
    }
}

}

namespace csf_cuda {

double reduceMaxDevice(const double* values, int count)
{
    if (count <= 0) {
        return 0.0;
    }

    int threads = 256;
    int blocks = (count + threads - 1) / threads;
    if (blocks > 1024) {
        blocks = 1024;
    }

    double* partial = 0;
    CSF_CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&partial), static_cast<std::size_t>(blocks) * sizeof(double)));
    reduceMaxKernel<<<blocks, threads, static_cast<std::size_t>(threads) * sizeof(double)>>>(values, partial, count);
    CSF_CUDA_CHECK(cudaGetLastError());

    if (blocks > 1) {
        reduceMaxKernel<<<1, threads, static_cast<std::size_t>(threads) * sizeof(double)>>>(partial, partial, blocks);
        CSF_CUDA_CHECK(cudaGetLastError());
    }

    double result = 0.0;
    CSF_CUDA_CHECK(cudaMemcpy(&result, partial, sizeof(double), cudaMemcpyDeviceToHost));
    CSF_CUDA_CHECK(cudaFree(partial));
    return result;
}

}
