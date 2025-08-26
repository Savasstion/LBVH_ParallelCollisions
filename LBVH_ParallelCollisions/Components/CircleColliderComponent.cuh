#pragma once
#include <set>
#include <vector>

#include "Transform2DComponent.h"
#include "../Base Classes/AABB.cuh"
#include "../Utillities/Vector2f.cuh"


struct CircleColliderComponent
{
    size_t entityID;
    bool isEventTrigger = false; // if false, physics response will be done, if true, only just call event trigger
    Vector2f relativePos = {0,0};
    float radius = .5f;

    AABB aabb;

    //  check if there are elements from current Set missing in prev Set
    //  if so, means exit collision event should occur for those elements
    std::set<size_t> currentTriggeredColliders;
    std::set<size_t> prevTriggeredColliders;

    //  contains all the collision objects to test exact collision (Separating Axis Theorem collision)
    //  shouldn't contain duplicate pairs of collision objects for all instance of collisionList in all ColliderComponent
    std::vector<size_t> collisionList;

    CircleColliderComponent(const Vector2f relativePos, const float radius)
        : relativePos(relativePos), radius(radius)
    {
        aabb = AABB(Vector2f(-radius, -radius), Vector2f(radius, radius));
    }

    CircleColliderComponent()
    {
        InitAABB();
    }

    void InitAABB();
    
    __host__ __device__ __forceinline__ AABB GetAABBInWorld(Vector2f position)
    {
        Vector2f transformedLower, transformedUpper;
        // Apply translation
        transformedLower.x = position.x + relativePos.x + aabb.lowerBound.x;
        transformedLower.y = position.y + relativePos.y + aabb.lowerBound.y;

        transformedUpper.x = position.x + relativePos.x+ aabb.upperBound.x;
        transformedUpper.y = position.y + relativePos.y+ aabb.upperBound.y;

        return AABB(transformedLower, transformedUpper);
    }
};
