#pragma once
#include "../Utillities/Vector2f.cuh"

struct Rigidbody2DComponent
{
    size_t entityID;
    bool isMoving = false;
    float mass = 1;
    float friction = 0;
    //  restitution is the ratio of how much a body will bounce back. If 1 then full bounce with no loss of energy. If 0, no bounce at all
    float restitution = 0.75;
    bool isStatic = false;
    
    Vector2f forceApplied = {0,0};
    Vector2f acceleration = {0,0};
    Vector2f velocity = {0,0};
    Vector2f maxVelocity = {1,1};

    bool isToggleGravity = true;

    Rigidbody2DComponent(const float mass, const float friction, const float restitution, const bool isStatic, const bool isToggleGravity)
        : mass(mass), friction(friction), restitution(restitution),isStatic(isStatic),isToggleGravity(isToggleGravity)
    {}

    Rigidbody2DComponent(){}

    void ApplyForce(Vector2f force);
};
