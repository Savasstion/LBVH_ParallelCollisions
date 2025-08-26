#pragma once
#include <vector>

#include "../Components/CircleColliderComponent.cuh"
#include "../Components/CircleRendererComponent.h"
#include "../Components/Rigidbody2DComponent.h"
#include "../Components/Transform2DComponent.h"

class ComponentManager
{
private:
    std::vector<TransformComponent> transform_components;
    std::vector<CircleRendererComponent> circle_renderer_components;
    std::vector<Rigidbody2DComponent> rigidbody_2d_components;
    std::vector<CircleColliderComponent> circle_collider_components;

public:
    size_t CreateTransformComponent(size_t entityID);
    size_t CreateTransformComponent(size_t entityID, Vector2f pos, Vector2f scale, float rotation);
    size_t CreateCircleRendererComponent(size_t entityID);
    size_t CreateCircleRendererComponent(size_t entityID, RGBColor color, unsigned int noOfTriangles);
    size_t CreateRigidbody2DComponent(size_t entityID);
    size_t CreateRigidbody2DComponent(size_t entityID, float mass, float friction, float restitution, bool isStatic, bool isToggleGravity);
    size_t CreateCircleColliderComponent(size_t entityID);
    size_t CreateCircleColliderComponent(size_t entityID, Vector2f relativePos, float radius);

    ComponentManager()=default;
    
    std::vector<TransformComponent>& GetTransformComponents()
    {
        return transform_components;
    }

    TransformComponent& GetTransformComponent(size_t index)
    {
        return transform_components[index];
    }

    std::vector<CircleRendererComponent>& GetCircleRendererComponents()
    {
        return circle_renderer_components;
    }

    CircleRendererComponent& GetCircleRendererComponent(size_t index)
    {
        return circle_renderer_components[index];
    }
    
    std::vector<Rigidbody2DComponent>& GetRigidbody2DComponents()
    {
        return rigidbody_2d_components;
    }

    Rigidbody2DComponent& GetRigidbody2DComponent(size_t index)
    {
        return rigidbody_2d_components[index];
    }

    std::vector<CircleColliderComponent>& GetCircleColliderComponents()
    {
        return circle_collider_components;
    }

    CircleColliderComponent& GetCircleColliderComponent(size_t index)
    {
        return circle_collider_components[index];
    }
};
