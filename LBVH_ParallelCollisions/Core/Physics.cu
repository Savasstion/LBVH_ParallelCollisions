#include "Physics.cuh"
#include "omp.h"

#undef min
#undef max 

const float Physics::globalGravityConstant = 1.0f;
std::vector<BVH::Node> Physics::collisionObjects;
BVH Physics::bvh;
Physics::BVH_TYPE Physics::bvhType = Physics::MPI;
std::vector<Rigidbody2DComponent>* Physics::ptr_rigidbody_2d_components;
std::vector<CircleColliderComponent>* Physics::ptr_circle_collider_components;
std::vector<TransformComponent>* Physics::ptr_transform_components;
std::vector<Entity>* Physics::ptr_entities;

Scene* Physics::ptrCurrentScene;
const float Physics::sceneLeftBoundary   = -1.0f;
const float Physics::sceneRightBoundary  = 1.0f;
const float Physics::sceneBottomBoundary = -1.0f;
const float Physics::sceneTopBoundary    = 1.0f;

double Physics::total_runtime_buffer = 0.0;
int Physics::frameCount = 0;
double Physics::frame_runtime = 0;

double Physics::total_cuda_parallel_runtime_buffer = 0.0;
double Physics::total_omp_parallel_runtime_buffer = 0.0;
double Physics::total_mpi_parallel_runtime_buffer = 0.0;

namespace
{
    size_t transformCount = 0, colliderCount = 0, entityCount = 0, rgbCount = 0;
    bool isSetNewValues = true;

    //  Device pointers
    TransformComponent* d_ptr_transform_components;
    Entity* d_ptr_entities;
    CircleColliderComponent* d_ptr_circle_collider_components;
    Rigidbody2DComponent* d_ptr_rigidbody_2d_components;
    unsigned long long* d_collisionLists;
    unsigned long long* d_listCounts;    
}


void Physics::DoScenePhysics(Scene& scene, int framesToUpdate)
{
    ptrCurrentScene = &scene;
    ptr_rigidbody_2d_components = &scene.component_manager.GetRigidbody2DComponents();
    ptr_circle_collider_components = &scene.component_manager.GetCircleColliderComponents();
    ptr_transform_components = &scene.component_manager.GetTransformComponents();
    ptr_entities = &scene.entity_manager.GetEntityList();

    //  set CUDA threads per block and amount of blocks
    size_t threadsPerBlock = 256;
    size_t leafBlocks;
    
    
    for(int i =0;i<framesToUpdate;i++)
    {
        DoAllCycleOfMotion();
        HandleSceneBoundaryCollision();
    }

    double start_time = omp_get_wtime();

    //  BVH Collision Procedures
    switch(bvhType)
    {
    case SERIAL:
        UpdateBVHNodes();
        ConstructCollisionBVH();
        SetSceneColliderCollisionLists();
        HandleAllCollision();
        break;
    case CUDA:
        CUDA_AllocateDeviceMemory();
        CUDA_SetDeviceMemory();
        leafBlocks = (colliderCount + threadsPerBlock - 1) / threadsPerBlock;
        UpdateBVHNodes();
        CUDA_ConstructCollisionBVHInParallel();
        CUDA_SetSceneColliderCollisionLists(leafBlocks, threadsPerBlock);
        CUDA_HandleAllCollision(leafBlocks, threadsPerBlock);
        CUDA_UpdateHostMemory();
        CUDA_FreeDeviceMemory();
        break;
    case OMP:
        UpdateBVHNodes();
        OMP_ConstructBVHInParallel();
        OMP_SetSceneColliderCollisionListsInParallel();
        OMP_HandleAllCollision();
        break;
    default:
        UpdateBVHNodes();
        ConstructCollisionBVH();
        SetSceneColliderCollisionLists();
        HandleAllCollision();
    }
    
    double end_time = omp_get_wtime();
    double frame_runtime = end_time - start_time;
    total_runtime_buffer += frame_runtime;
    frameCount++;
    
}

//void Physics::PrintFrameStats() {
//    std::cout << "Total frames: " << frameCount << "\n";
//    for (int i = 0; i < total_runtime_buffer.size(); ++i) {
//        std::cout << "Frame " << i << " runtime: " << total_runtime_buffer[i] << "s\n";
//    }
//}

void Physics::WriteAverageFrameTimeToFile(const std::string& filename) {
    if (frameCount == 0) return;

    // Calculate average frame time
    double average = total_runtime_buffer / frameCount;

    // Log average frame time for each method (CUDA, MPI, OpenMP)
    double average_cuda_time = total_cuda_parallel_runtime_buffer / frameCount;
    double average_omp_time = total_omp_parallel_runtime_buffer / frameCount;
    double average_mpi_time = total_mpi_parallel_runtime_buffer / frameCount;

    // Number of processors for each (you can tweak or detect automatically later)
    int cuda_cores = 3072; // RTX 4060
    int omp_threads = omp_get_max_threads(); // OpenMP cores
    int mpi_processes = 4; // assuming 4 MPI processes (adjust if different)

    // Calculate P values
    double P_cuda = average_cuda_time / average;
    double P_omp = average_omp_time / average;
    double P_mpi = average_mpi_time / average;

    // Calculate Amdahl's Law speedup
    double S_cuda = 1.0 / ((1.0 - P_cuda) + (P_cuda / cuda_cores));
    double S_omp = 1.0 / ((1.0 - P_omp) + (P_omp / omp_threads));
    double S_mpi = 1.0 / ((1.0 - P_mpi) + (P_mpi / mpi_processes));

    // Write data to the file
    std::ofstream outFile(filename);
    if (outFile.is_open()) {
        outFile << "Average frame time: " << average * 1000.0 << " milliseconds" << std::endl;
        outFile << "Total runtime: " << total_runtime_buffer << " seconds" << std::endl;
        outFile << "Total frames: " << frameCount << std::endl;
        outFile << std::endl;

        // CUDA info
        outFile << "[CUDA]\n";
        outFile << "CUDA average frame time: " << average_cuda_time * 1000.0 << " milliseconds\n";
        outFile << "CUDA total runtime: " << total_cuda_parallel_runtime_buffer << " seconds\n";
        outFile << "CUDA Parallel Fraction (P): " << P_cuda << "\n";
        outFile << "CUDA Theoretical Speedup (S): " << S_cuda << "\n\n";

        // OpenMP info
        outFile << "[OpenMP]\n";
        outFile << "OpenMP average frame time: " << average_omp_time * 1000.0 << " milliseconds\n";
        outFile << "OpenMP total runtime: " << total_omp_parallel_runtime_buffer << " seconds\n";
        outFile << "OpenMP Parallel Fraction (P): " << P_omp << "\n";
        outFile << "OpenMP Theoretical Speedup (S): " << S_omp << "\n\n";

        // MPI info
        outFile << "[MPI]\n";
        outFile << "MPI average frame time: " << average_mpi_time * 1000.0 << " milliseconds\n";
        outFile << "MPI total runtime: " << total_mpi_parallel_runtime_buffer << " seconds\n";
        outFile << "MPI Parallel Fraction (P): " << P_mpi << "\n";
        outFile << "MPI Theoretical Speedup (S): " << S_mpi << "\n\n";

        outFile.close();
    }
    else {
        std::cerr << "Failed to open file for writing average frame time." << std::endl;
    }
}

void Physics::DoAllCycleOfMotion()
{
    //  do Rigidbody2D cycle of motion
    for (Rigidbody2DComponent& c : *ptr_rigidbody_2d_components)
    {
        DoCycleOfMotion2D(c);
    }
}

void Physics::DoCycleOfMotion2D(Rigidbody2DComponent& rgb)
{
    if(!rgb.isStatic)
    {
        if (rgb.mass != 0.0f) {
            rgb.acceleration.x = rgb.forceApplied.x / rgb.mass;
            rgb.acceleration.y = rgb.forceApplied.y / rgb.mass;
        } else {
            rgb.acceleration = {0, 0}; // Prevent division by zero
        }
    
        // Vector2f frictionForce = CalculateFrictionForce(rgb.velocity, rgb.friction, rgb.mass);
        //
        // rgb.acceleration.x += frictionForce.x / rgb.mass;
        // rgb.acceleration.y += frictionForce.y / rgb.mass;

        rgb.velocity.x += rgb.acceleration.x;
        rgb.velocity.y += rgb.acceleration.y;
    
        //  caps velocity to max velocity
        if (sqrt((rgb.velocity.x * rgb.velocity.x) + (rgb.velocity.y * rgb.velocity.y)) > sqrt((rgb.maxVelocity.x * rgb.maxVelocity.x) + (rgb.maxVelocity.y * rgb.maxVelocity.y)))
        {
            // Normalize the velocity
            float velocityLength = sqrt((rgb.velocity.x * rgb.velocity.x) + (rgb.velocity.y * rgb.velocity.y));
            rgb.velocity.x /= velocityLength;
            rgb.velocity.y /= velocityLength;
            
            // Scale the velocity by max velocity length
            float maxVelocityLength = sqrt((rgb.maxVelocity.x * rgb.maxVelocity.x) + (rgb.maxVelocity.y * rgb.maxVelocity.y));
            rgb.velocity.x *= maxVelocityLength;
            rgb.velocity.y *= maxVelocityLength;
        }

        if (sqrt((rgb.velocity.x * rgb.velocity.x) + (rgb.velocity.y * rgb.velocity.y)) != 0.0f)
            rgb.isMoving = true;
        else
            rgb.isMoving = false;

        // Update the position if parent exists
        auto& entity = ptr_entities->at(rgb.entityID);
        auto& transform = ptr_transform_components->at(entity.transformComponentID);
        transform.position.x += rgb.velocity.x;
        transform.position.y += rgb.velocity.y;

        // Reset the force applied on entity
        rgb.forceApplied = {0,0};
    }
}

void Physics::HandleAllCollision()
{
    //  Check for Polygon2D collision
    //  Check collision lists of all colliders of which is obtained via the BVH
    CheckCollisionLists(*ptr_circle_collider_components);
}

void Physics::CUDA_HandleAllCollision(size_t blocksAmount, size_t threadsPerBlock)
{
    auto start = omp_get_wtime();

    CUDA_CheckCollisionListsInParallel<<<blocksAmount, threadsPerBlock>>>(d_collisionLists, d_listCounts, d_ptr_circle_collider_components, d_ptr_entities, d_ptr_transform_components, d_ptr_rigidbody_2d_components, colliderCount);
    cudaDeviceSynchronize();

    auto end = omp_get_wtime();
    total_cuda_parallel_runtime_buffer += (end - start);
}

void Physics::ResolveCollision(size_t entityAIndex, size_t entityBIndex)
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
    tA.position = tA.position - normalizedABVec * (overlap * .5f);
    tB.position = tB.position + normalizedABVec * (overlap * .5f);

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
    rgbA.velocity = rgbA.velocity + deltaVelocityA;

    //  B resolution
    Vector2f deltaVelocityB = distanceVec;
    deltaVelocityB = deltaVelocityB * (-2 * rgbA.mass * numerator / denominator);
    rgbB.velocity = rgbB.velocity + deltaVelocityB;
}

void Physics::HandleSceneBoundaryCollision()
{
    for(size_t i = 0; i < ptr_rigidbody_2d_components->size(); i++)
    {
        size_t entityID = ptr_rigidbody_2d_components->at(i).entityID;
        Entity& entity = ptr_entities->at(entityID);
        size_t transformID = entity.transformComponentID;
        TransformComponent& transform = ptr_transform_components->at(transformID);

        // Horizontal boundary
        if (transform.position.x > sceneRightBoundary - transform.scale.x) {
            transform.position.x = sceneRightBoundary - transform.scale.x;
            ptr_rigidbody_2d_components->at(i).velocity.x *= -1;
        }
        else if (transform.position.x < sceneLeftBoundary + transform.scale.x) {
            transform.position.x = sceneLeftBoundary + transform.scale.x;
            ptr_rigidbody_2d_components->at(i).velocity.x *= -1;
        }

        // Vertical boundary
        if (transform.position.y > sceneTopBoundary - transform.scale.x) {
            transform.position.y = sceneTopBoundary - transform.scale.x;
            ptr_rigidbody_2d_components->at(i).velocity.y *= -1;
        }
        else if (transform.position.y < sceneBottomBoundary + transform.scale.x) {
            transform.position.y = sceneBottomBoundary + transform.scale.x;
            ptr_rigidbody_2d_components->at(i).velocity.y *= -1;
        }
    }
}

bool Physics::CheckIfCirclesCollide(size_t entityAIndex, size_t entityBIndex)
{
    auto& cA = ptr_circle_collider_components->at(ptr_entities->at(entityAIndex).circleColliderComponentID);
    auto& cB = ptr_circle_collider_components->at(ptr_entities->at(entityBIndex).circleColliderComponentID);
    auto& ta = ptr_transform_components->at(ptr_entities->at(entityAIndex).transformComponentID);
    auto& tb = ptr_transform_components->at(ptr_entities->at(entityBIndex).transformComponentID); 
    float distance = (tb.position - ta.position).Magnitude();

    return distance < (cA.radius + cB.radius);
}

void Physics::UpdateBVHNodes()
{
    collisionObjects.clear();
    collisionObjects.reserve(ptr_circle_collider_components->size());

    for(size_t i = 0; i < ptr_circle_collider_components->size(); i++)
    {
        auto& c = ptr_circle_collider_components->at(i);
        Entity& entity = ptr_entities->at(c.entityID);
        TransformComponent& transform = ptr_transform_components->at(entity.transformComponentID);
        collisionObjects.emplace_back(BVH::Node(i, c.GetAABBInWorld(transform.position)));
        
    }
}

void Physics::ConstructCollisionBVH()
{
    bvh.ConstructBVH(collisionObjects);
}

void Physics::CUDA_ConstructCollisionBVHInParallel()
{
    auto start = omp_get_wtime();

    bvh.ConstructBVHInParallel(collisionObjects, *ptr_transform_components, *ptr_entities, *ptr_circle_collider_components);
    cudaDeviceSynchronize();

    auto end = omp_get_wtime();
    total_cuda_parallel_runtime_buffer += (end - start);
}

void Physics::SetSceneColliderCollisionLists()
{
    for(size_t i = 0; i < ptr_circle_collider_components->size(); i++)
    {
        auto& colliderComponent = ptr_circle_collider_components->at(i);
        colliderComponent.collisionList.clear();

        Entity& entity = ptr_entities->at(ptr_circle_collider_components->at(i).entityID);
        TransformComponent& transform = ptr_transform_components->at(entity.transformComponentID);

        bvh.TraverseRecursive(colliderComponent.collisionList, colliderComponent.GetAABBInWorld(transform.position), i, bvh.rootIndex);
    }
}

void Physics::CUDA_SetSceneColliderCollisionLists(size_t blocksAmount, size_t threadsPerBlock)
{
    auto start = omp_get_wtime();\

    CUDA_SetCollisionListsInParallel<<<blocksAmount, threadsPerBlock>>>(bvh.GetDeviceNodePointer(), d_collisionLists, d_listCounts, d_ptr_transform_components,d_ptr_entities, d_ptr_circle_collider_components, colliderCount, bvh.rootIndex);
    cudaDeviceSynchronize();

    auto end = omp_get_wtime();
    total_cuda_parallel_runtime_buffer += (end - start);
}

__global__ void CUDA_SetCollisionListsInParallel(BVH::Node* ptr_nodes, unsigned long long* collisionLists, unsigned long long* listCounts, TransformComponent* ptr_transform_components, Entity* ptr_entities, CircleColliderComponent* ptr_circle_collider_components, size_t objectCount, size_t rootIndex)
{
    size_t idx = threadIdx.x + blockIdx.x * blockDim.x;

    if(idx < objectCount)
    {
        auto&& colliderComponent = ptr_circle_collider_components[idx];
        
        Entity& entity = ptr_entities[ptr_circle_collider_components[idx].entityID];
        TransformComponent& transform = ptr_transform_components[entity.transformComponentID];
        
        TraverseInParallel(ptr_nodes, collisionLists, listCounts, colliderComponent.GetAABBInWorld(transform.position), idx, rootIndex, objectCount);
    }
}

__global__ void CUDA_CheckCollisionListsInParallel(unsigned long long* collisionLists, unsigned long long* listSizeCounts, CircleColliderComponent* ptr_circle_collider_components, Entity* ptr_entities, TransformComponent* ptr_transform_components, Rigidbody2DComponent* ptr_rigidbody_2d_components, size_t objectCount)
{
    size_t idx = threadIdx.x + blockIdx.x * blockDim.x;
    
    if(idx < objectCount)
    {
        CircleColliderComponent& colliderA = ptr_circle_collider_components[idx];
        size_t listSize = listSizeCounts[idx];
        
        for(size_t i = 0; i < listSize; i++)
        {
            size_t colliderBIndex = collisionLists[idx * (objectCount - 1) + i];
            CircleColliderComponent& colliderB = ptr_circle_collider_components[colliderBIndex];
            
            
            auto isCollided = CUDA_CheckIfCirclesCollideInDevice(colliderA.entityID, colliderB.entityID, ptr_circle_collider_components, ptr_entities, ptr_transform_components); //    Do circles collision test
            //  do collision routine
            if(isCollided)
            {
                //  Ensures no duplicate pairs do the same collision twice
                if (colliderA.entityID < colliderB.entityID)
                {
                    //  Add forces for realistic physics response
                    CUDA_ResolveCollisionInDevice(colliderA.entityID, colliderB.entityID, ptr_rigidbody_2d_components, ptr_entities, ptr_transform_components, ptr_circle_collider_components);
                }
            }
        }
    }
}


void Physics::CheckCollisionLists(std::vector<CircleColliderComponent>& components)
{
    for(size_t i = 0; i < components.size(); i++)
    {
        auto colliderA = components[i];
        std::vector<size_t>& collisionList = colliderA.collisionList;
        
        for(const auto& index : collisionList)
        {
            auto colliderB = components[index];
            
            auto isCollided = CheckIfCirclesCollide(colliderA.entityID,colliderB.entityID); //    Do circles collision test
            //  do collision routine
            if(isCollided)
            {
                //  Add forces for realistic physics response
                ResolveCollision(colliderA.entityID, colliderB.entityID);
                   
            }
        }
    }
}

void Physics::CUDA_AllocateDeviceMemory()
{
    auto currentTransformCount = ptr_transform_components->size();
    auto currentColliderCount = ptr_circle_collider_components->size();
    auto currentEntityCount = ptr_entities->size();
    auto currentRigidbodyCount = ptr_rigidbody_2d_components->size();

    if(currentTransformCount > transformCount || currentColliderCount > colliderCount || currentEntityCount > entityCount || currentRigidbodyCount > rgbCount)
    {
        cudaMalloc((void**)&d_ptr_transform_components, currentTransformCount * sizeof(TransformComponent));
        cudaMalloc((void**)&d_ptr_entities, currentEntityCount * sizeof(Entity));
        cudaMalloc((void**)&d_ptr_circle_collider_components, currentColliderCount * sizeof(CircleColliderComponent));
        cudaMalloc((void**)&d_ptr_rigidbody_2d_components, currentRigidbodyCount * sizeof(Rigidbody2DComponent));
        
        
        cudaMalloc((void**)&d_collisionLists, currentColliderCount * (currentColliderCount - 1) * sizeof(unsigned long long));
        cudaMalloc((void**)&d_listCounts, currentColliderCount * sizeof(unsigned long long));
    }

    transformCount = currentTransformCount;
    colliderCount = currentColliderCount;
    entityCount = currentEntityCount;
    rgbCount = currentRigidbodyCount;
    
}

void Physics::CUDA_SetDeviceMemory()
{
    if(isSetNewValues)
    {
        cudaMemcpy(d_ptr_entities, ptr_entities->data(), entityCount * sizeof(Entity), cudaMemcpyHostToDevice);
        cudaMemcpy(d_ptr_circle_collider_components, ptr_circle_collider_components->data(), colliderCount * sizeof(CircleColliderComponent), cudaMemcpyHostToDevice);
        isSetNewValues = false;
    }

    //  Transforms will change every scene
    cudaMemcpy(d_ptr_transform_components, ptr_transform_components->data(), transformCount * sizeof(TransformComponent), cudaMemcpyHostToDevice);

    //  Need to reset how much is the actual size of collisionList for each collider
    cudaMemset(d_listCounts, 0, sizeof(unsigned long long) * colliderCount);
    cudaMemset(d_collisionLists, 0, sizeof(unsigned long long) * colliderCount * (colliderCount - 1));
    
    //  Velocity values will change often between frames
    cudaMemcpy(d_ptr_rigidbody_2d_components, ptr_rigidbody_2d_components->data(), rgbCount * sizeof(Rigidbody2DComponent), cudaMemcpyHostToDevice);
}

void Physics::CUDA_UpdateHostMemory()
{
    if(rgbCount > 0)
    {
        cudaMemcpy(ptr_rigidbody_2d_components->data(), d_ptr_rigidbody_2d_components, rgbCount * sizeof(Rigidbody2DComponent), cudaMemcpyDeviceToHost);
    }

    if(transformCount > 0)
    {
        cudaMemcpy(ptr_transform_components->data(), d_ptr_transform_components, transformCount * sizeof(TransformComponent), cudaMemcpyDeviceToHost);
    }

}

void Physics::CUDA_FreeDeviceMemory()
{
    transformCount = 0;
    colliderCount = 0;
    entityCount = 0;
    rgbCount = 0;

    cudaFree(d_ptr_transform_components);
    cudaFree(d_ptr_entities);
    cudaFree(d_ptr_circle_collider_components);
    cudaFree(d_ptr_rigidbody_2d_components);
    cudaFree(d_collisionLists);
    cudaFree(d_listCounts);

    isSetNewValues = true;
    
}


bool CUDA_CheckIfCirclesCollideInDevice(size_t entityAIndex, size_t entityBIndex, CircleColliderComponent* ptr_circle_collider_components, Entity* ptr_entities, TransformComponent* ptr_transform_components)
{
    auto& cA = ptr_circle_collider_components[ptr_entities[entityAIndex].circleColliderComponentID];
    auto& cB = ptr_circle_collider_components[ptr_entities[entityBIndex].circleColliderComponentID];
    auto& ta = ptr_transform_components[ptr_entities[entityAIndex].transformComponentID];
    auto& tb = ptr_transform_components[ptr_entities[entityBIndex].transformComponentID]; 
    float distance = (tb.position - ta.position).Magnitude();

    return distance < (cA.radius + cB.radius);
}

__device__ void CUDA_ResolveCollisionInDevice(size_t entityAIndex, size_t entityBIndex, Rigidbody2DComponent* ptr_rigidbody_2d_components, Entity* ptr_entities, TransformComponent* ptr_transform_components, CircleColliderComponent* ptr_circle_collider_components)
{
    auto& rgbA = ptr_rigidbody_2d_components[ptr_entities[entityAIndex].rigidbody2DComponentID];
    auto& rgbB = ptr_rigidbody_2d_components[ptr_entities[entityBIndex].rigidbody2DComponentID];
    auto& tA = ptr_transform_components[ptr_entities[entityAIndex].transformComponentID];
    auto& tB = ptr_transform_components[ptr_entities[entityBIndex].transformComponentID]; 
    auto& cA = ptr_circle_collider_components[ptr_entities[entityAIndex].circleColliderComponentID];
    auto& cB = ptr_circle_collider_components[ptr_entities[entityBIndex].circleColliderComponentID];

    Vector2f distanceVec = tB.position - tA.position;
    float distance = distanceVec.Magnitude();
    Vector2f normalizedABVec = distanceVec.Normalize();

    float overlap = (cA.radius + cB.radius) - distance;

    //  move the two circles away from each other to eliminate overlap
    //tA.position = tA.position - normalizedABVec * (overlap * .5f);
    //tB.position = tB.position + normalizedABVec * (overlap * .5f);
    Vector2f newPosA = tA.position - normalizedABVec * (overlap * .5f);
    Vector2f newPosB = tB.position + normalizedABVec * (overlap * .5f);
    atomicExch(&tA.position.x, newPosA.x);
    atomicExch(&tA.position.y, newPosA.y);
    atomicExch(&tB.position.x, newPosB.x);
    atomicExch(&tB.position.y, newPosB.y);
    
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
    //rgbA.velocity = rgbA.velocity + deltaVelocityA;
    Vector2f newVelocityA = rgbA.velocity + deltaVelocityA;
    atomicExch(&rgbA.velocity.x, newVelocityA.x);
    atomicExch(&rgbA.velocity.y, newVelocityA.y);

    //  B resolution
    Vector2f deltaVelocityB = distanceVec;
    deltaVelocityB = deltaVelocityB * (-2 * rgbA.mass * numerator / denominator);
    //rgbB.velocity = rgbB.velocity + deltaVelocityB;
    Vector2f newVelocityB = rgbB.velocity + deltaVelocityB;
    atomicExch(&rgbB.velocity.x, newVelocityB.x);
    atomicExch(&rgbB.velocity.y, newVelocityB.y);
}
