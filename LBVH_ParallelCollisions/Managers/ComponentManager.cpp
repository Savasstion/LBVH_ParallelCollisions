#include "ComponentManager.h"

size_t ComponentManager::CreateTransformComponent(size_t entityID)
{
    transform_components.emplace_back();
    transform_components.back().entityID = entityID;

    return transform_components.size() - 1;
}

size_t ComponentManager::CreateTransformComponent(size_t entityID, Vector2f pos, Vector2f scale, float rotation)
{
    transform_components.emplace_back(pos, scale, rotation);
    transform_components.back().entityID = entityID;

    return transform_components.size() - 1;
}

size_t ComponentManager::CreateCircleRendererComponent(size_t entityID)
{
    circle_renderer_components.emplace_back();
    circle_renderer_components.back().entityID = entityID;

    return circle_renderer_components.size() - 1;
}

size_t ComponentManager::CreateCircleRendererComponent(size_t entityID, RGBColor color, unsigned int noOfTriangles)
{
    circle_renderer_components.emplace_back(color, noOfTriangles);
    circle_renderer_components.back().entityID = entityID;

    return circle_renderer_components.size() - 1;
}

size_t ComponentManager::CreateRigidbody2DComponent(size_t entityID)
{
    rigidbody_2d_components.emplace_back();
    rigidbody_2d_components.back().entityID = entityID;

    return rigidbody_2d_components.size() - 1;
}

size_t ComponentManager::CreateRigidbody2DComponent(size_t entityID, float mass, float friction, float restitution,
    bool isStatic, bool isToggleGravity)
{
    rigidbody_2d_components.emplace_back(mass, friction, restitution, isStatic,isToggleGravity);
    rigidbody_2d_components.back().entityID = entityID;

    return rigidbody_2d_components.size() - 1;
}

size_t ComponentManager::CreateCircleColliderComponent(size_t entityID)
{
    circle_collider_components.emplace_back();
    circle_collider_components.back().entityID = entityID;

    return circle_collider_components.size() - 1;
}

size_t ComponentManager::CreateCircleColliderComponent(size_t entityID, Vector2f relativePos, float radius)
{
    circle_collider_components.emplace_back(relativePos, radius);
    circle_collider_components.back().entityID = entityID;

    return circle_collider_components.size() - 1;
}
