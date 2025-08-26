#pragma once
#include "../Utillities/RGBColor.h"

struct CircleRendererComponent
{
    size_t entityID;
    RGBColor color = RGBColor();
    unsigned int noOfTriangles = 10;

    // Constructor to initialize with entity ID and color
    CircleRendererComponent(const RGBColor& color, unsigned int triangles)
        : color(color), noOfTriangles(triangles)
    {}

    CircleRendererComponent()
        : color(RGBColor()), noOfTriangles(10)
    {}

    // Set the color of the circle
    void SetColor(const RGBColor& newColor)
    {
        color = newColor;
    }

    // Set the number of triangles (for more precision or performance)
    void SetNumberOfTriangles(unsigned int triangles)
    {
        noOfTriangles = triangles;
    }
};
