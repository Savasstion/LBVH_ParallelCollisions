#pragma once
#include "../Utillities/Vector2f.cuh"
#include <math.h>

struct TransformComponent
{
    size_t entityID;
    Vector2f position;
    Vector2f scale;
    float rotation;

    // Default constructor
    TransformComponent() : position(0, 0), scale(1, 1), rotation(0) {}

    // Constructor with initial position, scale, and rotation
    TransformComponent(Vector2f pos, Vector2f sc, float rot)
        : position(pos), scale(sc), rotation(rot) {}

    // Apply translation (move the object)
    void Translate(const Vector2f& translation)
    {
        position = position + translation;
    }

    // Apply scaling
    void ScaleBy(const Vector2f& scaleFactor)
    {
        scale = scale * scaleFactor;
    }

    // Apply rotation (in radians)
    void Rotate(float angle)
    {
        rotation += angle;
        if (rotation >= 360.0f)
            rotation -= 360.0f;
        else if (rotation < 0.0f)
            rotation += 360.0f;
    }

    // Get the transformation matrix (2D)
    // This is a simple 2D transformation matrix [scale, rotate, translate]
    void GetTransformationMatrix(float matrix[6]) const
    {
        double PI = 3.14159265358979323846;
        float rad = rotation * (PI / 180.0f); // Convert rotation to radians
        float cosTheta = cos(rad);
        float sinTheta = sin(rad);

        // Set the matrix values (2x3 matrix in row-major order)
        matrix[0] = cosTheta * scale.x; // Scaling + Rotation (X)
        matrix[1] = sinTheta * scale.x; // Rotation (Y)
        matrix[2] = 0;                  // No shear
        matrix[3] = -sinTheta * scale.y; // Rotation (X)
        matrix[4] = cosTheta * scale.y;  // Scaling + Rotation (Y)
        matrix[5] = 0;                  // No shear

        matrix[2] = position.x; // Translation (X)
        matrix[5] = position.y; // Translation (Y)
    }
};
