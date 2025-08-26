#include "EntityManager.h"


size_t EntityManager::CreateEntity()
{
    entityList.emplace_back();
    
    return entityList.back().id;
}
