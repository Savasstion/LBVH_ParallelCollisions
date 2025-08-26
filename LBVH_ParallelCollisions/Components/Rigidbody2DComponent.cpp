#include "Rigidbody2DComponent.h"

void Rigidbody2DComponent::ApplyForce(Vector2f force)
{
    forceApplied.x += force.x;
    forceApplied.y += force.y;
}
