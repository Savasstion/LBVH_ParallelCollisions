#pragma once
#include <vector_types.h>
#include "../Base Classes/AABB.cuh"
#include "../Components/CircleColliderComponent.cuh"
#include <omp.h>


struct Entity;
struct TransformComponent;

class BVH
{
public:
    struct MortonCodeEntry 
    {
        unsigned int mortonCode;
        size_t objectIndex;
    };

    struct Node
    {
        AABB box;
        size_t objectIndex = -1;
        //int parentIndex;
        size_t child1 = -1;
        size_t child2 = -1;
        bool isLeaf;    //  false = internal node/sector, true = leaf node/collision object

        __host__ __device__ Node()
            : box(AABB()), objectIndex(-1), child1(-1), child2(-1), isLeaf(false) {}
        __host__ __device__ Node(const size_t objectIndex, const AABB& box)
            : box(box), objectIndex(objectIndex), child1(-1), child2(-1), isLeaf(true) {}
                
        __host__ __device__ Node(const size_t leftChild, const size_t rightChild, const AABB& box)
            : box(box), objectIndex(-1), child1(leftChild), child2(rightChild), isLeaf(false) {}
        __host__ __device__ Node(const size_t leftChild, const size_t rightChild)
            : objectIndex(-1), child1(leftChild), child2(rightChild), isLeaf(false), box(AABB()){}
        
    };
    
    //  Serial members
    std::vector<Node> s_nodes;
    
    size_t rootIndex = -1;

    const std::vector<Node>& GetNodes() const { return nodes; }
    std::vector<Node>& GetNodes() { return nodes; }

    
    size_t BuildHierarchy(std::vector<Node>& objects, size_t start, size_t end);
    size_t BuildHierarchyInParallel(Node* objects, TransformComponent* ptr_transform_components, Entity* ptr_entities, CircleColliderComponent* ptr_circle_collider_components, size_t objectCount);

    void ConstructBVH(std::vector<Node>& objects);
    void ConstructBVHInParallel(std::vector<Node>& objects, std::vector<TransformComponent>& transform_components, std::vector<Entity>& entities,
        std::vector<CircleColliderComponent>& circle_collider_components);
    void ClearBVH();
    void TraverseRecursive(std::vector<size_t>& collisionList, const AABB& queryAABB, size_t objectQueryIndex,size_t nodeIndex);
    Node* GetDeviceNodePointer();
    Node* GetHostNodePointer();
    void CUDA_SortMortonCodes(size_t objectCount);

    //OMP
    void OMP_ClearBVH();
    void OMP_ConstructBVHInParallel(std::vector<Node>& objects, std::vector<TransformComponent>& transform_components, std::vector<Entity>& entities,
                                           std::vector<CircleColliderComponent>& circle_collider_components);
    size_t OMP_BuildHierarchyInParallel(std::vector<Node>& objects, std::vector<TransformComponent>& transform_components, std::vector<Entity>& entities,
                                               std::vector<CircleColliderComponent>& circle_collider_components, size_t objectCount);

    //MPI
    void MPI_ClearBVH();
    void MPI_ConstructBVHInParallel(std::vector<Node>& objects, std::vector<TransformComponent>& transform_components, std::vector<Entity>& entities,
        std::vector<CircleColliderComponent>& circle_collider_components);
    size_t MPI_BuildHierarchyInParallel(std::vector<Node>& objects, std::vector<TransformComponent>& transform_components, std::vector<Entity>& entities,
        std::vector<CircleColliderComponent>& circle_collider_components, size_t objectCount);

private:
    std::vector<Node> nodes;
};
//  OMP
void OMP_AssignMortonCodes(TransformComponent* ptr_transform_components, Entity* ptr_entities,
                                        CircleColliderComponent* ptr_circle_collider_components, size_t objectCount);
void OMP_BuildLeafNodes(BVH::Node* ptr_nodes, size_t objectCount);
void OMP_BuildInternalNodes(BVH::Node* ptr_nodes, size_t objectCount);


//  MPI
void MPI_CreateBVHDataTypes();
void MPI_AssignMortonCodes(TransformComponent* ptr_transform_components, Entity* ptr_entities,
    CircleColliderComponent* ptr_circle_collider_components, size_t objectCount);
void MPI_BuildLeafNodes(BVH::Node* ptr_nodes, size_t objectCount);
void MPI_BuildInternalNodes(BVH::Node* ptr_nodes, size_t objectCount);

//  CUDA
__global__ void BuildLeafNodesKernel(BVH::MortonCodeEntry* ptr_sortedMortonCodes, BVH::Node* ptr_nodes, AABB* ptr_objectAABBs, size_t objectCount);
__global__ void BuildInternalNodesKernel(BVH::MortonCodeEntry* ptr_sortedMortonCodes, BVH::Node* ptr_nodes, size_t objectCount);
__global__ void AssignMortonCodesKernel(BVH::MortonCodeEntry* d_ptr_sortedMortonCodes, CircleColliderComponent* d_ptr_circle_collider_components, Entity* d_ptr_entities, TransformComponent* d_ptr_transform_components, size_t objectCount);
__global__ void SortMortonCodesKernel(BVH::MortonCodeEntry* data, int n);
//  Only uses one thread, as it only does serial code within gpu because we are acting on device memory as the device to reduce the amount time spent on sending data between host and device
//__global__ void AssignInternalNodeAABBKernel(size_t nodeIndex, BVH::Node* nodes, size_t nodeCount);

__host__ void AssignInternalNodeAABB(size_t nodeIndex, BVH::Node* nodes, size_t nodeCount);
__host__ void AssignInternalNodeAABB_Updated(BVH::Node* nodes, size_t objectCount);
__host__ __device__ int findSplit(BVH::MortonCodeEntry* morton, int first, int last);
__host__ __device__ int2 determineRange(BVH::MortonCodeEntry* p_sortedMortonCodes, int objectCount, int idx);

__device__ __forceinline__ void TraverseInParallel(BVH::Node* ptr_nodes, unsigned long long* collisionLists, unsigned long long* listSizeCounts, const AABB& queryAABB, size_t objectQueryIndex, size_t nodeIndex, size_t objectCount)
{
    // Stack to store nodes for iterative traversal
    int stack[256]; // You can adjust the stack size as needed
    int stackPointer = 0;

    // Push the root node onto the stack
    stack[stackPointer++] = nodeIndex;

    while (stackPointer > 0)
    {
        // Pop a node from the stack
        nodeIndex = stack[--stackPointer];

        // Check if overlap
        if (!AABB::isIntersect(queryAABB, ptr_nodes[nodeIndex].box))
            continue;

        if (ptr_nodes[nodeIndex].isLeaf)
        {
            size_t otherObjectIndex = ptr_nodes[nodeIndex].objectIndex;

            // Avoid self-collision
            if (objectQueryIndex == otherObjectIndex)
                continue;
            
            // Add to the collision list
            collisionLists[objectQueryIndex * (objectCount - 1) + listSizeCounts[objectQueryIndex]] = ptr_nodes[nodeIndex].objectIndex;
            listSizeCounts[objectQueryIndex] += 1;
        }
        else
        {
            // Push child nodes to the stack
            stack[stackPointer++] = ptr_nodes[nodeIndex].child1;
            stack[stackPointer++] = ptr_nodes[nodeIndex].child2;
        }
    }
}


//__host__ void TraverseInParallel(BVH::Node* ptr_nodes, std::vector<size_t>& collisionList, const AABB& queryAABB, size_t objectQueryIndex, size_t nodeIndex);
__host__ void CUDA_AllocateMemory(size_t colliderComponentCount, size_t transformComponentCount, size_t entityComponentCount);
__host__ void CUDA_CopyComponentsFromHostToDevice(BVH::Node* objects, TransformComponent* ptr_transform_components, Entity* ptr_entities, CircleColliderComponent* ptr_circle_collider_components);
__host__ void CUDA_FreeDeviceSpaceForBVH();


