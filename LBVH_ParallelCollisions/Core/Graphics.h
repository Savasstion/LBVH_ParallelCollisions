#pragma once
#include "../Base Classes/Scene.h"

class Graphics
{
public:
    static const float sceneTopBoundary;
    static const float sceneLeftBoundary;
    static const float sceneRightBoundary;
    static const float sceneBottomBoundary;
    static void Render(Scene& scene);
    static void DrawSolidCircle(float centerX, float centerY, float radius, int noOfTriangles, float red, float green, float blue);
};
