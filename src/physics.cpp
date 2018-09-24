// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org/>
#include "physics.h"

namespace physics
{
    static float2 cross(float a, float2 b) { return cross(float3{0,0,a},float3{b,0}).xy(); }
    static float2 cross(float2 a, float b) { return cross(float3{a,0},float3{0,0,b}).xy(); }

    float2 rigidbody::velocity() const { return momentum*inv_mass; }
    float rigidbody::spin() const { return angular_momentum*inv_moment;}
    float2 rigidbody::velocity_at_arm(const float2 & arm) const { return velocity() + cross(spin(), arm); }
    void rigidbody::apply_impulse_at_arm(const float2 & arm, const float2 & impulse)
    {
        momentum += impulse;
        angular_momentum += cross(arm, impulse);
    }

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
                float impulse_denom = c.body_a->inv_mass + dot(cross(c.body_a->inv_moment * cross(c.arm_a, c.normal_a_to_b), c.arm_a), c.normal_a_to_b)
                                    + (c.body_b ? c.body_b->inv_mass + dot(cross(c.body_b->inv_moment * cross(c.arm_b, c.normal_a_to_b), c.arm_b), c.normal_a_to_b) : 0);
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