#pragma once
#include "../Managers/ComponentManager.h"
#include "../Managers/EntityManager.h"

class Scene
{
public:
    EntityManager entity_manager;
    ComponentManager component_manager;

    Scene(): entity_manager(EntityManager()), component_manager(ComponentManager()){}
};
