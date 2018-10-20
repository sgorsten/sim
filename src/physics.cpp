// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org/>
#include "physics.h"
#include <algorithm>

namespace physics
{
    mass_distribution compute_mass_for_circle(float density, float radius)
    {
        const float area = radius*radius*3.141593f;
        const float mass = area*density;
        const float moment = mass*radius*radius/2;
        return {mass, 1/mass, 1/moment};
    }

    mass_distribution compute_mass_for_box(float density, float2 dims)
    {
        const float area = dims.x * dims.y;
        const float mass = area*density;
        const float moment = mass*length2(dims)/12;
        return {mass, 1/mass, 1/moment};
    }

    float2 rigidbody::velocity() const { return momentum*mass_dist.inv_mass; }
    float rigidbody::spin() const { return angular_momentum*mass_dist.inv_moment;}
    float2 rigidbody::velocity_at_arm(const float2 & arm) const { return velocity() + cross(spin(), arm); }
    void rigidbody::apply_impulse_at_arm(const float2 & arm, const float2 & impulse)
    {
        momentum += impulse;
        angular_momentum += cross(arm, impulse);
    }

    static float sqr(float x) { return x*x; }
    void solve_constraints(const std::vector<linear_constraint> & constraints)
    {
        std::vector<float> constraint_impulses(constraints.size(), 0.0f);

        for(int i=0; i<10; ++i)
        {
            for(size_t i=0; i<constraints.size(); ++i)
            {
                auto & c = constraints[i];
                auto & sum = constraint_impulses[i];

                // Determine relative velocity
                float2 v0 = c.body_a->velocity_at_arm(c.arm_a);
                float2 v1 = c.body_b ? c.body_b->velocity_at_arm(c.arm_b) : float2{0,0};
                float vn = dot(v1-v0, c.normal_a_to_b);

                // Determine impulse needed to achieve target velocity, and clamp it against impulse limits
                float impulse_denom = c.body_a->mass_dist.inv_mass + c.body_a->mass_dist.inv_moment * sqr(dot(cross(1.0f,c.arm_a), c.normal_a_to_b))
                                    + (c.body_b ? c.body_b->mass_dist.inv_mass + c.body_b->mass_dist.inv_moment * sqr(dot(cross(1.0f,c.arm_b), c.normal_a_to_b)) : 0);
                float impulse = (c.target_velocity - vn) / impulse_denom;
                impulse = std::max(impulse, c.min_impulse - sum);
                impulse = std::min(impulse, c.max_impulse - sum);

                // Apply impulse and record it in the totals
                float2 impulse_vec = c.normal_a_to_b * impulse;
                c.body_a->apply_impulse_at_arm(c.arm_a, -impulse_vec);
                if(c.body_b) c.body_b->apply_impulse_at_arm(c.arm_b, impulse_vec);
                sum += impulse;
            }
        }
    }
}