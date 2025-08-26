#pragma once
#include <vector>
#include "../Base Classes/Entity.h"

class EntityManager
{
public:
    std::vector<Entity> entityList;

    size_t CreateEntity();
    std::vector<Entity>& GetEntityList(){return entityList;}
    Entity& GetEntity(size_t id){return entityList[id];}
};
