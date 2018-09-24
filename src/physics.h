// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org/>
#pragma once
#include <vector>
#include "linalg.h"
using namespace linalg::aliases;

namespace physics
{
    struct rigidbody 
    { 
        float2 position, momentum;
        float orientation, angular_momentum;
        float inv_mass, inv_moment, elasticity; 
    
        float2 velocity() const;
        float spin() const;

        // arm refers to displacement vector from rigidbody origin in world orientation
        float2 velocity_at_arm(const float2 & arm) const;
        void apply_impulse_at_arm(const float2 & arm, const float2 & impulse);
    };

    struct linear_constraint
    {
        rigidbody * body_a;         // Non-nullable, there must always be at least one rigidbody referenced by the constraint
        rigidbody * body_b;         // Nullable, in the case where a rigidbody is constrained to the world itself
        float2 arm_a, arm_b;        // Displacement vector on each body where constraint should act
        float2 normal_a_to_b;       // Unit length vector from body A to body B

        float target_velocity;  // The intended velocity along this limit (zero for ball joints, resting contacts, nonzero for elastic collisions)
        float min_impulse;      // The minimum amount of impulse that can be applied (zero for contacts, -inf for ball joints, etc)
        float max_impulse;      // The maximum amount of impulse that can be applied (+inf for ball joints, etc)
    };

    void solve_constraints(const std::vector<linear_constraint> & constraints);
}