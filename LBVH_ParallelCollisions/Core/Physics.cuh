#pragma once
#include <vector>
#include "../Utillities/Vector2f.cuh"
#include "../Data Structures/BVH.cuh"
#include "../Base Classes/Scene.h"
#include <iostream>
#include <fstream>
#include <omp.h>

class Physics
{
public:
    enum BVH_TYPE
    {
        SERIAL,
        CUDA,
        OMP,
        MPI
    };
    static BVH_TYPE bvhType;
    static const float globalGravityConstant;
    static const float sceneLeftBoundary;  
    static const float sceneRightBoundary;
    static const float sceneBottomBoundary; 
    static const float sceneTopBoundary;
    
    static void DoScenePhysics(Scene& scene, int framesToUpdate);
    //  Motion
    static Vector2f CalculateFrictionForce(Vector2f velocity, float friction, float mass);
    //  Collision
    static bool CheckIfPolygons2DIntersect(std::vector<Vector2f> verticesA, std::vector<Vector2f> verticesB, Vector2f* normal, float* depth);
    //  MPI
    static void Start_MPI();
    static void Finalize_MPI();

    static int frameCount;
    static double frame_runtime;

    static void WriteAverageFrameTimeToFile(const std::string& filename);
    static double total_runtime_buffer;

    static double total_cuda_parallel_runtime_buffer;
    static double total_omp_parallel_runtime_buffer;
    static double total_mpi_parallel_runtime_buffer;

private:
    static std::vector<BVH::Node> collisionObjects; // note that the node's index is the same as the corresponding collider's index in the collider component array
    static BVH bvh;
    static std::vector<Rigidbody2DComponent>* ptr_rigidbody_2d_components;
    static std::vector<CircleColliderComponent>* ptr_circle_collider_components;
    static std::vector<TransformComponent>* ptr_transform_components;
    static std::vector<Entity>* ptr_entities;
    static Scene* ptrCurrentScene;
    
    //  Motion
    static void DoAllCycleOfMotion();    
    static void DoCycleOfMotion2D(Rigidbody2DComponent& rgb);
    //  Collision
    static void HandleAllCollision();
    static void CUDA_HandleAllCollision(size_t blocksAmount, size_t threadsPerBlock);
    static void HandleSceneBoundaryCollision();
    static bool CheckIfCirclesCollide(size_t entityAIndex, size_t entityBIndex);
    static void ResolveCollision(size_t entityAIndex, size_t entityBIndex);
    // BVH
    static void UpdateBVHNodes();
    static void ConstructCollisionBVH();
    static void CUDA_ConstructCollisionBVHInParallel();
    static void SetSceneColliderCollisionLists();
    static void CUDA_SetSceneColliderCollisionLists(size_t blocksAmount, size_t threadsPerBlock);
    static void CheckCollisionLists(std::vector<CircleColliderComponent>& components);

    static void CUDA_AllocateDeviceMemory();
    static void CUDA_SetDeviceMemory();
    static void CUDA_UpdateHostMemory();
    static void CUDA_FreeDeviceMemory();


    //OMP
    static inline void OMP_ConstructBVHInParallel()
    {
        bvh.OMP_ConstructBVHInParallel(collisionObjects, *ptr_transform_components, *ptr_entities, *ptr_circle_collider_components);
    }
    static void OMP_SetSceneColliderCollisionListsInParallel();
    static void OMP_HandleAllCollision();
    static void OMP_CheckCollisionLists(std::vector<CircleColliderComponent>& components);
    static void OMP_ResolveCollision(size_t entityAIndex, size_t entityBIndex);

    static void OMP_HandleCollisionsParallel(std::vector<CircleColliderComponent>& local_circle_colliders);
    static void GatherCollisionResultsMPI();
    static void BroadcastUpdatedCollisions();
    
};

__device__ bool CUDA_CheckIfCirclesCollideInDevice(size_t entityAIndex, size_t entityBIndex, CircleColliderComponent* ptr_circle_collider_components, Entity* ptr_entities, TransformComponent* ptr_transform_components);
__device__ void CUDA_ResolveCollisionInDevice(size_t entityAIndex, size_t entityBIndex, Rigidbody2DComponent* ptr_rigidbody_2d_components, Entity* ptr_entities, TransformComponent* ptr_transform_components, CircleColliderComponent* ptr_circle_collider_components);

__global__ void CUDA_SetCollisionListsInParallel(BVH::Node* ptr_nodes, unsigned long long* collisionLists, unsigned long long* listCounts, TransformComponent* ptr_transform_components, Entity* ptr_entities, CircleColliderComponent* ptr_circle_collider_components, size_t objectCount, size_t rootIndex);
__global__ void CUDA_CheckCollisionListsInParallel(unsigned long long* collisionLists, unsigned long long* listSizeCounts, CircleColliderComponent* ptr_circle_collider_components, Entity* ptr_entities, TransformComponent* ptr_transform_components, Rigidbody2DComponent* ptr_rigidbody_2d_components, size_t objectCount);

