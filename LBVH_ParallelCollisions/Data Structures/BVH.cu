#include "BVH.cuh"
#include <algorithm>
#include <vector_types.h>
#include "../Base Classes/Entity.h"
#include "../Utillities/MortonCode.cuh"
#include <cstdlib>
#include <iostream>

#include "../Utillities/BitManipulation.cuh"


//  members for parallelization
namespace
{
    BVH::Node* d_ptr_nodes;
    BVH::Node* h_ptr_nodes;
    BVH::Node* d_ptr_collisionObjects;
    BVH::MortonCodeEntry* h_ptr_sortedMortonCodes;
    BVH::MortonCodeEntry* d_ptr_sortedMortonCodes;
    AABB* d_ptr_objectAABBs;

    CircleColliderComponent* d_ptr_circle_collider_components;
    TransformComponent* d_ptr_transform_components;
    Entity* d_ptr_entities;

    size_t nodeCount;
    bool isNewValuesSet = true;
    size_t colliderCount = 0, transformCount = 0, entityCount = 0;
}


size_t BVH::BuildHierarchy(std::vector<Node>& objects, size_t start, size_t end)
{
    //get all AABBs of all objects
    std::vector<AABB> objectAABBs;
    objectAABBs.reserve(objects.size());
    
    for(const auto& obj : objects)
    {
        objectAABBs.emplace_back(obj.box);
    }
    
    if (start == end)
    {
        s_nodes.emplace_back(start, objectAABBs[start]);  // Leaf node
        return s_nodes.size() - 1;
    }

    // Compute bounding box for all objects in the range
    AABB bounds = objectAABBs[start];
    for (size_t i = start + 1; i <= end; i++)
    {
        bounds = AABB::UnionAABB(bounds, objectAABBs[i]);
    }

    // Simple partition: Split at the middle
    size_t mid = (start + end) / 2;
    std::nth_element(objectAABBs.begin() + start, objectAABBs.begin() + mid, objectAABBs.begin() + end + 1,
                     [](const AABB& a, const AABB& b) { return a.lowerBound.x < b.lowerBound.x; });

    size_t leftChild = BuildHierarchy(objects, start, mid);
    size_t rightChild = BuildHierarchy(objects, mid + 1, end);

    s_nodes.emplace_back(leftChild, rightChild, bounds);  // Internal node
    return s_nodes.size() - 1;
}

size_t BVH::BuildHierarchyInParallel(Node* objects, TransformComponent* ptr_transform_components, Entity* ptr_entities, CircleColliderComponent* ptr_circle_collider_components, size_t objectCount)
{
    //  set CUDA threads per block and amount of blocks
    size_t threadsPerBlock = 256;
    size_t leafBlocks = (objectCount + threadsPerBlock - 1) / threadsPerBlock;
    size_t internalBlocks = ((objectCount - 1) + threadsPerBlock - 1) / threadsPerBlock;
    
    CUDA_CopyComponentsFromHostToDevice(objects, ptr_transform_components, ptr_entities, ptr_circle_collider_components);
    
    // Assign Morton Codes
    AssignMortonCodesKernel<<<leafBlocks, threadsPerBlock >>>(d_ptr_sortedMortonCodes, d_ptr_circle_collider_components, d_ptr_entities, d_ptr_transform_components, objectCount);
    cudaDeviceSynchronize();

    //  TODO : Actually implement a parallel binary radix sorting algorithm
    CUDA_SortMortonCodes(objectCount);
    //cudaDeviceSynchronize();
    
    //  first N nodes in p_ptr_nodes are leafs, the next N - 1 nodes will be internal nodes
    BuildLeafNodesKernel<<<leafBlocks, threadsPerBlock>>>(d_ptr_sortedMortonCodes, d_ptr_nodes, d_ptr_objectAABBs, objectCount);
    cudaDeviceSynchronize();
    BuildInternalNodesKernel<<<internalBlocks, threadsPerBlock>>>(d_ptr_sortedMortonCodes, d_ptr_nodes, objectCount);
    cudaDeviceSynchronize();

    //  Use leaf nodes's bounding boxes to get each internal nodes's encompassing bounding boxes
    //cudaMemcpy(h_ptr_nodes, d_ptr_nodes, nodeCount * sizeof(Node), cudaMemcpyDeviceToHost);
    //AssignInternalNodeAABB_Updated(h_ptr_nodes, colliderCount);
    //AssignInternalNodeAABB(rootIndex, h_ptr_nodes, nodeCount);
    //cudaMemcpy(d_ptr_nodes, h_ptr_nodes, nodeCount * sizeof(Node), cudaMemcpyHostToDevice);
    
    return objectCount; //root always N
}



void BVH::ConstructBVH(std::vector<Node>& objects)
{
    //  Simple and naive implementation
    ClearBVH();
    if (!objects.empty())
        rootIndex = BuildHierarchy(objects, 0, objects.size() - 1);
}

void BVH::ConstructBVHInParallel(std::vector<Node>& objects, std::vector<TransformComponent>& transform_components, std::vector<Entity>& entities,
    std::vector<CircleColliderComponent>& circle_collider_components)
{
    ClearBVH();
    size_t objectCount = circle_collider_components.size();
    CUDA_AllocateMemory(circle_collider_components.size(), transform_components.size(), entities.size());

    //Build Hierarchy
    if (objectCount > 0)
    {
        rootIndex = BuildHierarchyInParallel(objects.data(), transform_components.data(), entities.data(), circle_collider_components.data(), objectCount);
        
    }
        
}


void BVH::ClearBVH()
{
    s_nodes.clear();
    rootIndex = -1;
    
    free(h_ptr_nodes);
    h_ptr_nodes = nullptr;
    CUDA_FreeDeviceSpaceForBVH();

}

void BVH::TraverseRecursive(std::vector<size_t>& collisionList, const AABB& queryAABB, size_t objectQueryIndex, size_t nodeIndex)
{
    if (nodeIndex >= s_nodes.size()) {
        std::cerr << "[ERROR] nodeIndex out of bounds: " << nodeIndex << " (size: " << s_nodes.size() << ")\n";
        return;
    }

    const auto& node = s_nodes[nodeIndex];

    // Check if overlap
    if (!AABB::isIntersect(queryAABB, node.box))
        return;

    if (node.isLeaf)
    {
        if (node.objectIndex == objectQueryIndex)
            return;

        collisionList.emplace_back(node.objectIndex);
    }
    else
    {
        // Debug checks for child bounds
        if (node.child1 >= s_nodes.size() || node.child2 >= s_nodes.size()) {
            std::cerr << "[ERROR] Invalid child index at node " << nodeIndex
                << " | child1: " << node.child1 << ", child2: " << node.child2
                << ", s_nodes.size(): " << s_nodes.size() << "\n";
            return;
        }

        TraverseRecursive(collisionList, queryAABB, objectQueryIndex, node.child1);
        TraverseRecursive(collisionList, queryAABB, objectQueryIndex, node.child2);
    }
}



// Original
//void BVH::TraverseRecursive(std::vector<size_t>& collisionList, const AABB& queryAABB, size_t objectQueryIndex, size_t nodeIndex)
//{
//    // Check if overlap
//    if (!AABB::isIntersect(queryAABB, s_nodes[nodeIndex].box))
//        return;
//    
//    if (s_nodes[nodeIndex].isLeaf)
//    {
//        size_t otherObjectIndex = s_nodes[nodeIndex].objectIndex;
//
//        // Avoid self-collision
//        if (objectQueryIndex == otherObjectIndex)
//            return;
//        
//        collisionList.emplace_back(s_nodes[nodeIndex].objectIndex);
//    }
//    else
//    {
//        // Traverse child nodes
//        TraverseRecursive(collisionList, queryAABB, objectQueryIndex, s_nodes[nodeIndex].child1);
//        TraverseRecursive(collisionList, queryAABB, objectQueryIndex, s_nodes[nodeIndex].child2);
//    }
//}

BVH::Node* BVH::GetDeviceNodePointer()
{
    return d_ptr_nodes;
}

BVH::Node* BVH::GetHostNodePointer()
{
    return h_ptr_nodes;
}


__global__ void AssignMortonCodesKernel(BVH::MortonCodeEntry* d_ptr_sortedMortonCodes, CircleColliderComponent* d_ptr_circle_collider_components, Entity* d_ptr_entities, TransformComponent* d_ptr_transform_components, size_t objectCount)
{
    //  Serial
    /*for(size_t i = 0 ; i < objectCount; i++)
    {
        d_ptr_sortedMortonCodes[i].objectIndex = i;

        auto eID = ptr_circle_collider_components[i].entityID;
        auto tID = ptr_entities[eID].transformComponentID;
        auto&& transform = ptr_transform_components[tID];
        d_ptr_sortedMortonCodes[i].mortonCode = morton2D(transform.position.x, transform.position.y);
    }*/

    //Parallel
    size_t idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= objectCount) return;
    
    d_ptr_sortedMortonCodes[idx].objectIndex = idx;
    
    auto eID = d_ptr_circle_collider_components[idx].entityID;
    auto tID = d_ptr_entities[eID].transformComponentID;
    auto& transform = d_ptr_transform_components[tID];
    d_ptr_sortedMortonCodes[idx].mortonCode = morton2D(transform.position.x, transform.position.y);
    
}

__global__ void SortMortonCodesKernel(BVH::MortonCodeEntry* data, int n)
{
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        // Simple insertion sort for MortonCodeEntry
        for (int i = 1; i < n; ++i) {
            BVH::MortonCodeEntry key = data[i];
            int j = i - 1;

            while (j >= 0 && data[j].mortonCode > key.mortonCode) {
                data[j + 1] = data[j];
                j--;
            }
            data[j + 1] = key;
        }
    }
}

__host__ void AssignInternalNodeAABB(size_t nodeIndex, BVH::Node* nodes, size_t nodeCount)
{

    if (nodeIndex >= nodeCount) return;
    
    BVH::Node& node = nodes[nodeIndex];
    if (node.isLeaf) return;
    
    AABB* leftAABB = new AABB(); 
    AABB* rightAABB = new AABB();
    
    if (node.child1 != -1 && node.child1 < nodeCount)
    {
        AssignInternalNodeAABB(node.child1, nodes, nodeCount);
        *leftAABB  = nodes[node.child1].box;
    }
    else
        *leftAABB = AABB();
    
    if (node.child2 != -1 && node.child2 < nodeCount)
    {
        AssignInternalNodeAABB(node.child2, nodes, nodeCount);
        *rightAABB = nodes[node.child2].box;
    }
    else
        *rightAABB = AABB();
    
    node.box = AABB::UnionAABB(*leftAABB, *rightAABB);
    
    free(leftAABB);
    free(rightAABB);
}


void AssignInternalNodeAABB_Updated(BVH::Node* nodes, size_t objectCount)
{
    //printf("AssignInternalNodeAABB running on CPU...\n");
    for(size_t i = 0; i < (objectCount - 1); i++)
    {
        int2 range = determineRange(h_ptr_sortedMortonCodes, objectCount, static_cast<int>(i));

        //  get all encompassing leaf node's AABBs within the range
        AABB internalNodeBox = nodes[range.x].box;
        for (int j = range.x + 1; j <= range.y; j++)
        {
            if(nodes[j].isLeaf)
                internalNodeBox = AABB::UnionAABB(internalNodeBox, nodes[j].box);
        }

        nodes[objectCount + i].box = internalNodeBox;
        //printf("Updated node : %d\n", objectCount + i);
    }
}


// __global__ void AssignInternalNodeAABBKernel(size_t nodeIndex, BVH::Node* nodes, size_t nodeCount)
// {
//     size_t idx = threadIdx.x + blockIdx.x * blockDim.x;
//     if (threadIdx.x != 0 || blockIdx.x != 0) return;
//     
//     if (nodeIndex >= nodeCount) return;
//     
//     BVH::Node& node = nodes[nodeIndex];
//     if (node.isLeaf) return;
//
//     AABB leftAABB; 
//     AABB rightAABB;
//     
//     if (node.child1 != -1 && node.child1 < nodeCount)
//     {
//         AssignInternalNodeAABB(node.child1, nodes, nodeCount);
//         leftAABB  = nodes[node.child1].box;
//     }
//     else
//         leftAABB = AABB();
//
//     if (node.child2 != -1 && node.child2 < nodeCount)
//     {
//         AssignInternalNodeAABB(node.child2, nodes, nodeCount);
//         rightAABB = nodes[node.child2].box;
//     }
//     else
//         rightAABB = AABB();
//
//     node.box = AABB::UnionAABB(leftAABB, rightAABB);
// }

void BVH::CUDA_SortMortonCodes(size_t objectCount)
{
    //  Take copy from device memory
    cudaMemcpy(h_ptr_sortedMortonCodes, d_ptr_sortedMortonCodes, objectCount * sizeof(MortonCodeEntry), cudaMemcpyDeviceToHost);
    
    std::sort(h_ptr_sortedMortonCodes, h_ptr_sortedMortonCodes + objectCount, [](const MortonCodeEntry& a, const MortonCodeEntry& b) {
        return a.mortonCode < b.mortonCode;
    });

    //  Copy back to device memory
    cudaMemcpy(d_ptr_sortedMortonCodes, h_ptr_sortedMortonCodes, objectCount * sizeof(MortonCodeEntry), cudaMemcpyHostToDevice);
}


__global__ void BuildLeafNodesKernel(BVH::MortonCodeEntry* ptr_sortedMortonCodes, BVH::Node* ptr_nodes, AABB* ptr_objectAABBs, size_t objectCount)
{
    size_t idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= objectCount) return;

    size_t objIndex = ptr_sortedMortonCodes[idx].objectIndex;
    ptr_nodes[idx] = BVH::Node(objIndex, ptr_objectAABBs[objIndex]); // Leaf constructor
    
    // Serial version
    // first N indices will be leaf nodes
    //for (size_t idx = 0; idx < objectCount; idx++) // in parallel
    //{
    //    //leafNodes[idx].objectID = sortedObjectIDs[idx];
    //    p_ptr_nodes[idx] = Node(p_sortedMortonCodes[idx].objectIndex, objectAABBs[p_sortedMortonCodes[idx].objectIndex]);   //  Leaf node constructor
    //}
}

__global__ void BuildInternalNodesKernel(BVH::MortonCodeEntry* ptr_sortedMortonCodes, BVH::Node* ptr_nodes, size_t objectCount)
{
    size_t idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= objectCount - 1) return;   
    
    int2 range = determineRange(ptr_sortedMortonCodes, objectCount, static_cast<int>(idx));
    int first = range.x;
    int last = range.y;

    //  get all encompassing leaf node's AABBs within the range
    AABB internalNodeBox = ptr_nodes[range.x].box;
    for (int i = range.x + 1; i <= range.y; i++)
    {
        if(ptr_nodes[i].isLeaf)
            internalNodeBox = AABB::UnionAABB(internalNodeBox, ptr_nodes[i].box);
    }
    
    int split = findSplit(ptr_sortedMortonCodes, first, last);

    size_t indexA = (split == first) ? split : objectCount + split;
    size_t indexB = (split + 1 == last) ? (split + 1) : objectCount + split + 1;

    ptr_nodes[idx + objectCount] = BVH::Node(indexA, indexB, internalNodeBox); // Internal constructor

    // Serial version
    //for (size_t idx = 0; idx < objectCount - 1; idx++) // in parallel
    //{
    //    // Find out which range of objects the node corresponds to.

    //    //  may need to convert idx to 32-bit int
    //    int2 range = determineRange(d_ptr_sortedMortonCodes, objectCount, idx);
    //    int first = range.x;
    //    int last = range.y;

    //    //get all encompassing leaf node's AABBs within the range
    // AABB internalNodeBox = ptr_nodes[range.x].box;
    // for (int i = range.x + 1; i <= range.y; i++)
    // {
    //     if(ptr_nodes[i].isLeaf)
    //         internalNodeBox = AABB::UnionAABB(internalNodeBox, ptr_nodes[i].box);
    // }

    //    // Determine where to split the range.
    //    int split = findSplit(d_ptr_sortedMortonCodes, first, last);

    //    // Select childA.
    //    size_t indexA;
    //    if (split == first)
    //    {
    //        indexA = split;
    //    }
    //    else
    //    {
    //        indexA = objectCount + split;
    //    }
    //    
    //    // Select childB.
    //    size_t indexB;
    //    if (split + 1 == last)
    //    {
    //        indexB = split + 1;
    //    }
    //    else
    //    {
    //        indexB = objectCount + split + 1;
    //    }
    //    
    //    // Record parent-child relationships.
    //    d_ptr_nodes[idx + objectCount] = Node(indexA, indexB, internalNodeBox);
    //}
}

__host__ __device__ int2 determineRange(BVH::MortonCodeEntry* p_sortedMortonCodes, int objectCount, int idx)
{
    if(idx == 0)
    {
        int2 range = { 0, objectCount-1};
        return range;
    }
    
    // Determine direction of the range
    unsigned int selfMortonCode = p_sortedMortonCodes[idx].mortonCode;
    int deltaL = clz(selfMortonCode ^ p_sortedMortonCodes[idx - 1].mortonCode);
    int deltaR = clz(selfMortonCode ^ p_sortedMortonCodes[idx + 1].mortonCode);
    int direction = (deltaR > deltaL) ? 1 : -1;
    

    // Compute upper bound for the length of the range
    int deltaMin = (deltaL < deltaR) ? deltaL : deltaR;
    int lmax = 2;
    int delta = -1;
    int i_tmp = idx + direction * lmax;
    
    if(0 <= i_tmp && i_tmp < objectCount)
    {
        delta = clz(selfMortonCode ^ p_sortedMortonCodes[i_tmp].mortonCode);
    }

    while(delta > deltaMin)
    {
        lmax <<= 1;
        i_tmp = idx + direction * lmax;
        delta = 1;
        if(0 <= i_tmp && i_tmp < objectCount)
        {
            delta = clz(selfMortonCode ^ p_sortedMortonCodes[i_tmp].mortonCode);
        }
    }
    
    // Find the other end using binary search
    int l = 0;
    int t = lmax >> 1;
    while(t > 0)
    {
        i_tmp = idx + (l + t) * direction;
        delta = -1;
        if(0 <= i_tmp && i_tmp < objectCount)
        {
            delta = clz(selfMortonCode ^ p_sortedMortonCodes[i_tmp].mortonCode);
        }
        if(delta > deltaMin)
        {
            l += t;
        }
        t >>= 1;
    }
    unsigned int jdx = idx + l * direction;
    if(direction < 0)
    {
        unsigned int tmp = idx;
        idx = jdx;
        jdx = tmp;
    }

    int2 result;
    result.x = idx;
    result.y = jdx;
    return result;
}

__host__ __device__ int findSplit(BVH::MortonCodeEntry* morton, int first, int last) 
{
    // Identical Morton codes => split the range in the middle.

    unsigned int firstCode = morton[first].mortonCode;
    unsigned int lastCode = morton[last].mortonCode;

    if (firstCode == lastCode)
        return (first + last) >> 1;

    // Calculate the number of highest bits that are the same
    // for all objects, using the count-leading-zeros intrinsic.

    int delta_node  = clz(firstCode ^ lastCode);

    // Use binary search to find where the next bit differs.
    // Specifically, we are looking for the highest object that
    // shares more than commonPrefix bits with the first one.

    int split = first; // initial guess
    int stride  = last - first;

    do
    {
        stride  = (stride  + 1) >> 1; // exponential decrease
        int middle = split + stride ; // proposed new position

        if (middle < last)
        {
            int delta = clz(firstCode ^ morton[middle].mortonCode);
            if (delta > delta_node)
                split = middle; // accept proposal
        }
    }
    while (stride > 1);

    return split;
}


void CUDA_AllocateMemory(size_t colliderComponentCount, size_t transformComponentCount, size_t entityTotalCount)
{
    // if no increase in amount of space needed, then no need to allocate
    if(colliderComponentCount > colliderCount || transformComponentCount > transformCount || entityTotalCount > entityCount)
    {
        nodeCount = 2 * colliderComponentCount - 1;
        cudaMalloc((void**)&d_ptr_nodes, nodeCount * sizeof(BVH::Node));
        h_ptr_nodes = (BVH::Node*)malloc(nodeCount * sizeof(BVH::Node));
        cudaMalloc((void**)&d_ptr_collisionObjects, colliderComponentCount * sizeof(BVH::Node));

        cudaMalloc((void**)&d_ptr_transform_components, transformComponentCount * sizeof(TransformComponent));
        cudaMalloc((void**)&d_ptr_circle_collider_components, colliderComponentCount * sizeof(CircleColliderComponent));
        cudaMalloc((void**)&d_ptr_entities, entityTotalCount * sizeof(Entity));
        h_ptr_sortedMortonCodes = (BVH::MortonCodeEntry*)malloc(nodeCount * sizeof(BVH::MortonCodeEntry));
        cudaMalloc((void**)&d_ptr_sortedMortonCodes, nodeCount * sizeof(BVH::MortonCodeEntry));
        cudaMalloc((void**)&d_ptr_objectAABBs, colliderComponentCount * sizeof(AABB));

        colliderCount = colliderComponentCount;
        transformCount = transformComponentCount;
        entityCount = entityTotalCount;
    }

}

__host__ void CUDA_CopyComponentsFromHostToDevice(BVH::Node* ptr_objects, TransformComponent* ptr_transform_components, Entity* ptr_entities, CircleColliderComponent* ptr_circle_collider_components)
{
    //  If unchanged, no need to copy same values again
    if(isNewValuesSet)
    {
        cudaMemcpy(d_ptr_collisionObjects, ptr_objects, colliderCount * sizeof(BVH::Node), cudaMemcpyHostToDevice);
        cudaMemcpy(d_ptr_entities, ptr_entities, entityCount * sizeof(Entity), cudaMemcpyHostToDevice);
        cudaMemcpy(d_ptr_circle_collider_components, ptr_circle_collider_components, colliderCount * sizeof(CircleColliderComponent), cudaMemcpyHostToDevice);
        
        isNewValuesSet = false;
    }

    //  positions will always change in this scene
    cudaMemcpy(d_ptr_transform_components, ptr_transform_components, transformCount * sizeof(TransformComponent), cudaMemcpyHostToDevice);
    
    //get all AABBs of all objects, leaf nodes may be in different order every frame
    AABB* temp_ptr_objectAABBs = (AABB*)malloc(colliderCount * sizeof(AABB));
    for (size_t i = 0; i < colliderCount; i++)
    {
        temp_ptr_objectAABBs[i] = (ptr_objects[i].box);
    }
    cudaMemcpy(d_ptr_objectAABBs, temp_ptr_objectAABBs, colliderCount * sizeof(AABB), cudaMemcpyHostToDevice);
    free(temp_ptr_objectAABBs);
    temp_ptr_objectAABBs = nullptr;
}

__host__ void CUDA_FreeDeviceSpaceForBVH()
{
    cudaFree(d_ptr_nodes);
    cudaFree(d_ptr_collisionObjects);
    cudaFree(d_ptr_transform_components);
    cudaFree(d_ptr_circle_collider_components);
    cudaFree(d_ptr_entities);
    cudaFree(d_ptr_sortedMortonCodes);
    cudaFree(d_ptr_objectAABBs);

    free(h_ptr_nodes);
    free(h_ptr_sortedMortonCodes);

    colliderCount = 0;
    transformCount = 0;
    entityCount = 0;

    isNewValuesSet = true;
}
