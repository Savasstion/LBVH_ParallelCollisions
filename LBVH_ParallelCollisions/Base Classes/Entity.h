#pragma once

struct Entity
{
    static size_t currentID;
    size_t id;
    bool isActive = true;

    size_t transformComponentID;
    size_t circleRendererComponentID;
    size_t rigidbody2DComponentID;
    size_t circleColliderComponentID;
    
    Entity(): id(currentID){currentID++;}
};
