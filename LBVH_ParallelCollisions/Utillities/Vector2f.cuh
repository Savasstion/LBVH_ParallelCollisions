#pragma once
#include <math.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

struct Vector2f
{
    float x, y;

    // Default constructor
    __host__ __device__ __forceinline__ Vector2f() : x(0), y(0) {}

    // Constructor with specific values
    __host__ __device__ __forceinline__ Vector2f(float x, float y) : x(x), y(y) {}

    // Add two vectors
    __host__ __device__ __forceinline__ Vector2f operator+(const Vector2f& other) const
    {
        return Vector2f(x + other.x, y + other.y);
    }

    // Subtract two vectors
    __host__ __device__ __forceinline__ Vector2f operator-(const Vector2f& other) const
    {
        return Vector2f(x - other.x, y - other.y);
    }

    // Scalar multiplication
    __host__ __device__ __forceinline__ Vector2f operator*(float scalar) const
    {
        return Vector2f(x * scalar, y * scalar);
    }

    // Vector multiplication
    __host__ __device__ __forceinline__ Vector2f operator*(Vector2f otherVec) const
    {
        return Vector2f(x * otherVec.x, y * otherVec.y);
    }

    // Scalar division
    __host__ __device__ __forceinline__ Vector2f operator/(float scalar) const
    {
        if (scalar != 0)
            return Vector2f(x / scalar, y / scalar);
        return *this; // Return the same vector if division by zero
    }


    // Vector division
    __host__ __device__ __forceinline__ Vector2f operator/(Vector2f otherVec) const
    {
        if (otherVec.x != 0 && otherVec.y != 0)
            return Vector2f(x / otherVec.x, y / otherVec.y);
        return *this; // Return the same vector if division by zero
    }


    // Dot product
    __host__ __device__ __forceinline__ float Dot(const Vector2f& other) const
    {
        return x * other.x + y * other.y;
    }


    // Magnitude (length) of the vector
    __host__ __device__ __forceinline__ float Magnitude() const
    {
        return sqrtf(x * x + y * y);
    }

    // Normalize the vector (make it a unit vector)
    __host__ __device__ __forceinline__ Vector2f Normalize() const
    {
        float mag = Magnitude();
        if (mag > 0)
            return *this / mag;
        return *this; // Return the same vector if magnitude is 0
    }

    // Perpendicular vector (90 degrees rotation counterclockwise)
    __host__ __device__ __forceinline__ Vector2f Perpendicular() const
    {
        return Vector2f(-y, x);
    }

    // Negate the vector
    __host__ __device__ __forceinline__ Vector2f operator-() const
    {
        return Vector2f(-x, -y);
    }

    // Overload the equality operator
    __host__ __device__ __forceinline__ bool operator==(const Vector2f& other) const
    {
        return x == other.x && y == other.y;
    }

    // Overload the inequality operator
    __host__ __device__ __forceinline__ bool operator!=(const Vector2f& other) const
    {
        return !(*this == other);
    }
};


