#include "Graphics.h"
#include <cmath>
#include <Windows.h>
#include <gl/GL.h>

const float Graphics::sceneLeftBoundary   = -1.0f;
const float Graphics::sceneRightBoundary  = 1.0f;
const float Graphics::sceneBottomBoundary = -1.0f;
const float Graphics::sceneTopBoundary    = 1.0f;

void Graphics::Render(Scene& scene)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    auto& components = scene.component_manager.GetCircleRendererComponents();

    for(int i = 0; i < components.size(); i++)
    {
        auto transform = scene.component_manager.GetTransformComponent(components[i].entityID);
        DrawSolidCircle(transform.position.x / ((-sceneLeftBoundary + sceneRightBoundary)/2),transform.position.y / ((-sceneBottomBoundary + sceneTopBoundary)/2),transform.scale.x,components[i].noOfTriangles,
            components[i].color.r,components[i].color.g,components[i].color.b);
    }
    
}

void Graphics::DrawSolidCircle(float centerX, float centerY, float radius, int noOfTriangles, float red, float green, float blue)
{
    const double PI = 3.14159265358979323846;
    
    glBegin(GL_TRIANGLE_FAN);
    glColor3f(red, green, blue);
    glVertex2f(centerX, centerY);  // The center of the circle

    for (int i = 0; i <= noOfTriangles; ++i)
    {
        // Calculate the angle for each vertex
        float angle = 2 * PI * i / noOfTriangles;
        
        // Calculate the point on the circumference
        float x2 = centerX + radius * cos(angle);
        float y2 = centerY + radius * sin(angle);
        
        // Draw the vertex
        glVertex2f(x2, y2);
    }

    glEnd();
}
