#pragma once

struct RGBColor
{
    //  range between 0-1
    float r,g,b;

    // Default constructor (sets to white)
    RGBColor() = default;
    
    // Constructor to initialize with values
    RGBColor(float r, float g, float b) : r(r), g(g), b(b) {}
    
    // Add two RGB colors
    RGBColor operator+(const RGBColor& other) const
    {
        return RGBColor(r + other.r, g + other.g, b + other.b);
    }
    
    // Subtract two RGB colors
    RGBColor operator-(const RGBColor& other) const
    {
        return RGBColor(r - other.r, g - other.g, b - other.b);
    }
    
    // Scale (multiply) the color by a scalar
    RGBColor operator*(float scalar) const
    {
        return RGBColor(r * scalar, g * scalar, b * scalar);
    }
    
    // Scale the color by another RGB (element-wise multiplication)
    RGBColor operator*(const RGBColor& other) const
    {
        return RGBColor(r * other.r, g * other.g, b * other.b);
    }
    
};
