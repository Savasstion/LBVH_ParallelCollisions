#pragma once
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "../Utillities/Vector2f.cuh"

struct AABB
{
    Vector2f lowerBound;    // Bottom-left corner
    Vector2f upperBound;    // Upper-right corner

    __host__ __device__ __forceinline__ AABB(const Vector2f lowerBound, const Vector2f upperBound)
        : lowerBound(lowerBound), upperBound(upperBound) {
    }
    __host__ __device__ __forceinline__ AABB()
        : lowerBound(Vector2f(0, 0)), upperBound(Vector2f(0, 0)) {
    }
    __host__ __device__ __forceinline__ static AABB UnionAABB(const AABB& a, const AABB& b)
    {
        AABB C;
        C.lowerBound = Vector2f(fminf(a.lowerBound.x, b.lowerBound.x),
            fminf(a.lowerBound.y, b.lowerBound.y));
        C.upperBound = Vector2f(fmaxf(a.upperBound.x, b.upperBound.x),
            fmaxf(a.upperBound.y, b.upperBound.y));
        return C;
    }
    __host__ __device__ __forceinline__ static bool isIntersect(const AABB& a, const AABB& b)
    {
        return !(a.upperBound.x < b.lowerBound.x ||
            a.lowerBound.x > b.upperBound.x ||
            a.upperBound.y < b.lowerBound.y ||
            a.lowerBound.y > b.upperBound.y);
    }
    __host__ __device__ __forceinline__ bool isIntersect(const AABB& other)
    {
        return AABB::isIntersect(*this, other);
    }
};
