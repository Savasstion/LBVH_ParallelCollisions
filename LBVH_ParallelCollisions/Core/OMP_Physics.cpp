#include "Physics.cuh"

void Physics::OMP_SetSceneColliderCollisionListsInParallel()
{
    double start_time = omp_get_wtime();

#pragma omp parallel for
    for (int i = 0; i < ptr_circle_collider_components->size(); i++)
    {
        auto& colliderComponent = ptr_circle_collider_components->at(i);
        colliderComponent.collisionList.clear();

        Entity& entity = ptr_entities->at(ptr_circle_collider_components->at(i).entityID);
        TransformComponent& transform = ptr_transform_components->at(entity.transformComponentID);

      

        bvh.TraverseRecursive(colliderComponent.collisionList, colliderComponent.GetAABBInWorld(transform.position), i, bvh.rootIndex);

    }

    double end_time = omp_get_wtime(); // End timing
    Physics::total_omp_parallel_runtime_buffer += (end_time - start_time);
}

void Physics::OMP_HandleAllCollision()
{
    double start_time = omp_get_wtime();


    OMP_CheckCollisionLists(*ptr_circle_collider_components);


    double end_time = omp_get_wtime();
    Physics::total_omp_parallel_runtime_buffer += (end_time - start_time);
}


void Physics::OMP_CheckCollisionLists(std::vector<CircleColliderComponent>& components)
{
#pragma omp parallel for
    for (int i = 0; i < components.size(); i++)
    {
        auto colliderA = components[i];
        std::vector<size_t>& collisionList = colliderA.collisionList;

        for (const auto& index : collisionList)
        {
            auto colliderB = components[index];

            auto isCollided = CheckIfCirclesCollide(colliderA.entityID, colliderB.entityID); //    Do circles collision test
            //  do collision routine
            if (isCollided)
            {
                //  Ensures no duplicate pairs do the same collision twice
                if (colliderA.entityID < colliderB.entityID)
                {
                    //  Add forces for realistic physics response
                    OMP_ResolveCollision(colliderA.entityID, colliderB.entityID);
                }

            }
        }
    }
}

void Physics::OMP_ResolveCollision(size_t entityAIndex, size_t entityBIndex)
{
    auto& rgbA = ptr_rigidbody_2d_components->at(ptr_entities->at(entityAIndex).rigidbody2DComponentID);
    auto& rgbB = ptr_rigidbody_2d_components->at(ptr_entities->at(entityBIndex).rigidbody2DComponentID);
    auto& tA = ptr_transform_components->at(ptr_entities->at(entityAIndex).transformComponentID);
    auto& tB = ptr_transform_components->at(ptr_entities->at(entityBIndex).transformComponentID);
    auto& cA = ptr_circle_collider_components->at(ptr_entities->at(entityAIndex).circleColliderComponentID);
    auto& cB = ptr_circle_collider_components->at(ptr_entities->at(entityBIndex).circleColliderComponentID);

    Vector2f distanceVec = tB.position - tA.position;
    float distance = distanceVec.Magnitude();
    Vector2f normalizedABVec = distanceVec.Normalize();

    float overlap = (cA.radius + cB.radius) - distance;

    //  move the two circles away from each other to eliminate overlap
#pragma omp critical(updateTA)
    {
        tA.position = tA.position - normalizedABVec * (overlap * .5f);
    }
#pragma omp critical(updateTB)
    {
        tB.position = tB.position + normalizedABVec * (overlap * .5f);
    }

    //  recalculate distance
    distanceVec = tB.position - tA.position;
    distance = distanceVec.Magnitude();

    float massSum = rgbA.mass + rgbB.mass;
    Vector2f velocityDiff = rgbB.velocity - rgbA.velocity;

    //  A resolution
    float numerator = velocityDiff.Dot(distanceVec);
    float denominator = massSum * distance * distance;
    Vector2f deltaVelocityA = distanceVec;
    deltaVelocityA = deltaVelocityA * (2 * rgbB.mass * numerator / denominator);
#pragma omp critical(updateVA)
    {
        rgbA.velocity = rgbA.velocity + deltaVelocityA;
    }

    //  B resolution
    Vector2f deltaVelocityB = distanceVec;
    deltaVelocityB = deltaVelocityB * (-2 * rgbA.mass * numerator / denominator);
#pragma omp critical(updateVB)
    {
        rgbB.velocity = rgbB.velocity + deltaVelocityB;
    }
}

void Physics::OMP_HandleCollisionsParallel(std::vector<CircleColliderComponent>& local_circle_colliders)
{
    // Each process performs collision detection in parallel using OpenMP
#pragma omp parallel for
    for (int i = 0; i < local_circle_colliders.size(); i++)
    {
        auto& colliderA = local_circle_colliders[i];
        std::vector<size_t>& collisionList = colliderA.collisionList;

        // Check and resolve collisions locally for each collider
        for (const auto& index : collisionList)
        {
            auto& colliderB = local_circle_colliders[index];

            // Check if the circles collide
            if (CheckIfCirclesCollide(colliderA.entityID, colliderB.entityID)) {
                // If colliding, resolve the collision
                OMP_ResolveCollision(colliderA.entityID, colliderB.entityID);
            }
        }
    }
}


