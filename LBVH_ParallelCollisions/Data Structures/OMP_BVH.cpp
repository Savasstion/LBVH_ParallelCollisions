#include <algorithm>

#include "BVH.cuh"
#include "../Base Classes/Entity.h"
#include "../Utillities/MortonCode.cuh"
#include "../Core/Physics.cuh"

//  OMP variable
namespace 
{
    std::vector<BVH::MortonCodeEntry>  omp_sortedMortonCodes;
    std::vector<AABB> omp_AABBs;
}


void BVH::OMP_ClearBVH()
{
    s_nodes.clear();
    omp_sortedMortonCodes.clear();
    omp_AABBs.clear();
    rootIndex = -1;
}

void BVH::OMP_ConstructBVHInParallel(std::vector<Node>& objects, std::vector<TransformComponent>& transform_components,
    std::vector<Entity>& entities, std::vector<CircleColliderComponent>& circle_collider_components)
{
    double start_time = omp_get_wtime();

    OMP_ClearBVH();
    if (!objects.empty())
        rootIndex = OMP_BuildHierarchyInParallel(objects, transform_components, entities, circle_collider_components, objects.size());

    double end_time = omp_get_wtime();
    Physics::total_omp_parallel_runtime_buffer += (end_time - start_time);
}

void OMP_AssignMortonCodes(TransformComponent* ptr_transform_components, Entity* ptr_entities,
    CircleColliderComponent* ptr_circle_collider_components, size_t objectCount)
{
    #pragma omp parallel for
    for(int i = 0 ; i < objectCount; i++)
    {
        omp_sortedMortonCodes[i].objectIndex = i;
    
        auto eID = ptr_circle_collider_components[i].entityID;
        auto tID = ptr_entities[eID].transformComponentID;
        auto&& transform = ptr_transform_components[tID];
        omp_sortedMortonCodes[i].mortonCode = morton2D(transform.position.x, transform.position.y);
    }
    
}

void OMP_BuildLeafNodes(BVH::Node* ptr_nodes, size_t objectCount)
{
    // first N indices will be leaf nodes
    #pragma omp parallel for
    for (int idx = 0; idx < objectCount; idx++) // in parallel
    {
        //leafNodes[idx].objectID = sortedObjectIDs[idx];
        ptr_nodes[idx] = BVH::Node(omp_sortedMortonCodes[idx].objectIndex, omp_AABBs[omp_sortedMortonCodes[idx].objectIndex]);   //  Leaf node constructor
    }
}

void OMP_BuildInternalNodes(BVH::Node* ptr_nodes, size_t objectCount)
{
    #pragma omp parallel for
    for (int idx = 0; idx < objectCount - 1; idx++) // in parallel
    {
        // Find out which range of objects the node corresponds to.

        //  may need to convert idx to 32-bit int
        int2 range = determineRange(omp_sortedMortonCodes.data(), objectCount, idx);
        int first = range.x;
        int last = range.y;
        
        //  get all encompassing leaf node's AABBs within the range
        AABB internalNodeBox = ptr_nodes[range.x].box;
        for (int i = range.x + 1; i <= range.y; i++)
        {
            if(ptr_nodes[i].isLeaf)
                internalNodeBox = AABB::UnionAABB(internalNodeBox, ptr_nodes[i].box);
        }

        // Determine where to split the range.
        int split = findSplit(omp_sortedMortonCodes.data(), first, last);

        // Select childA.
        size_t indexA;
        if (split == first)
        {
            indexA = split;
        }
        else
        {
            indexA = objectCount + split;
        }
        
        // Select childB.
        size_t indexB;
        if (split + 1 == last)
        {
            indexB = split + 1;
        }
        else
        {
            indexB = objectCount + split + 1;
        }
        
        // Record parent-child relationships.
        ptr_nodes[idx + objectCount] = BVH::Node(indexA, indexB, internalNodeBox);  
    }
}

size_t BVH::OMP_BuildHierarchyInParallel(std::vector<Node>& objects,
                                         std::vector<TransformComponent>& transform_components, std::vector<Entity>& entities,
                                         std::vector<CircleColliderComponent>& circle_collider_components, size_t objectCount)
{
    //get all AABBs of all objects, need them for leaf node initialization later
    omp_AABBs.assign(objectCount, AABB());
    #pragma omp parallel for
    for (int i = 0; i < objectCount; i++)
    {
        omp_AABBs[i] = (objects[i].box);
    }
    
    // Assign Morton Codes
    omp_sortedMortonCodes.assign(objectCount, MortonCodeEntry());
    OMP_AssignMortonCodes(transform_components.data(), entities.data(), circle_collider_components.data(), objectCount);

    //  Sort Morton Codes
    std::sort(omp_sortedMortonCodes.data(), omp_sortedMortonCodes.data() + objectCount, [](const MortonCodeEntry& a, const MortonCodeEntry& b) {
        return a.mortonCode < b.mortonCode;
    });

    //  Total bvh nodes will always be 2 * N - 1
    //  leaf node count = N
    //  internal node count = N - 1 
    s_nodes.assign(2 * objectCount - 1, Node());
    //  first N nodes in p_ptr_nodes are leafs, the next N - 1 nodes will be internal nodes
    OMP_BuildLeafNodes(s_nodes.data(), objectCount);
    OMP_BuildInternalNodes(s_nodes.data(), objectCount);

    
    return objectCount; //root always N
}
