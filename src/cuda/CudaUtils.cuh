#ifndef _CSF_CUDA_UTILS_CUH_
#define _CSF_CUDA_UTILS_CUH_

#include <cuda_runtime.h>
#include <sstream>
#include <stdexcept>
#include <string>

namespace csf_cuda {

inline void checkCuda(cudaError_t status,
                      const char* expression,
                      const char* file,
                      int line)
{
    if (status != cudaSuccess) {
        std::ostringstream out;
        out << "CUDA error at " << file << ":" << line << " while running "
            << expression << ": " << cudaGetErrorString(status);
        throw std::runtime_error(out.str());
    }
}

double reduceMaxDevice(const double* values, int count);

}

#define CSF_CUDA_CHECK(expr) \
    ::csf_cuda::checkCuda((expr), #expr, __FILE__, __LINE__)

#endif
