#pragma once
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

//  Count leading zeroes
__host__ __device__ __forceinline__ int clz(unsigned int x)
{
    if (x == 0) return 32; // All bits are zero

    int count = 0;
    for (unsigned int mask = 1U << 31; (x & mask) == 0; mask >>= 1) {
        ++count;
    }
    return count;
}

