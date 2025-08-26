#include "CircleColliderComponent.cuh"

void CircleColliderComponent::InitAABB()
{
    aabb = AABB(Vector2f(-radius, -radius), Vector2f(radius, radius));
}


