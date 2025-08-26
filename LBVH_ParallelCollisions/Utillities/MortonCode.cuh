#include "cuda_runtime.h"
#include <device_launch_parameters.h>

//Expands a 10-bit integer into 30 bits
//by inserting 2 zeros after each bit.
__device__ __host__ __forceinline__ unsigned int expandBits(unsigned int v)
{
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}
//given 2D point located within the unit square [0,1].
__device__ __host__ __forceinline__ unsigned int morton2D(float x, float y)
{
    //  Convert from -1 to 1 range to 0 to 1 range
    x = (x + 1.0f) * 0.5f;
    y = (y + 1.0f) * 0.5f;

    x = fminf(fmaxf(x * 1024.0f, 0.0f), 1023.0f);
    y = fminf(fmaxf(y * 1024.0f, 0.0f), 1023.0f);

    unsigned int xx = expandBits((unsigned int)x);
    unsigned int yy = expandBits((unsigned int)y);

    return (xx << 1) | yy;
}