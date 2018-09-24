// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org/>
#pragma once
#include <vector>
#include <optional>
#include "linalg.h"
using namespace linalg::aliases;

namespace collision
{
    struct penetration
    {
        float2 p,n; float d;
        float2 point_on_a() const { return p; }
        float2 point_on_b() const { return p-n*d; }
        float2 normal_a_to_b() const { return n; }
        float2 normal_b_to_a() const { return -n; }
        float penetration_depth() const { return d; }
    };

    template<class SupportFunctionA, class SupportFunctionB> bool check_intersection(SupportFunctionA support_a, SupportFunctionB support_b, float2 initial_direction);
    template<class SupportFunctionA, class SupportFunctionB> std::optional<penetration> find_intersection(SupportFunctionA support_a, SupportFunctionB support_b, float2 initial_direction, float epsilon=0.0001f);

    // Implementation details
    namespace detail
    {
        struct point { float2 p, point_on_a; };
        struct simplex { point points[3]; int count; };
        struct polytope_edge { point v0, v1; float2 normal; float distance; };
        std::tuple<simplex,float2> next_simplex(const simplex & s, const point & new_point);
        std::vector<polytope_edge> make_polytope(const simplex & s);
        bool expand_polytope(std::vector<polytope_edge> & edges, point point);
        penetration penetration_from_nearest_edge(const polytope_edge & edge);
        template<class SupportFunction> std::optional<simplex> find_intersection_simplex(SupportFunction support_a_minus_b, float2 initial_direction, float epsilon)
        {
            simplex s {{support_a_minus_b(initial_direction)},1};
            float2 direction = -s.points[0].p;
            while(true)
            {
                if(!(length2(direction) > 0)) direction = -initial_direction;
                const point p = support_a_minus_b(direction);
                if(dot(p.p, direction) < 0) return std::nullopt;
                for(int i=0; i<s.count; ++i) if(p.p == s.points[i].p) return std::nullopt; // If point is already in simplex, then we've gotten as close as we can get with no intersection
                std::tie(s, direction) = next_simplex(s, p);
                if(s.count == 3) return s;
            }
        }
        template<class SupportFunction> std::optional<penetration> find_intersection(SupportFunction support_a_minus_b, float2 initial_direction, float epsilon)
        {
            auto s = find_intersection_simplex(support_a_minus_b, initial_direction, epsilon);
            if(!s) return std::nullopt;
            auto edges = make_polytope(*s);
            while(true)
            {
                const auto it = std::min_element(begin(edges), end(edges), [](const polytope_edge & a, const polytope_edge & b) { return a.distance < b.distance; });
                if(edges.size() == 32) return penetration_from_nearest_edge(*it);
                const point p = support_a_minus_b(it->normal);
                if(dot(p.p, it->normal) <= it->distance + epsilon) return penetration_from_nearest_edge(*it);
                if(!expand_polytope(edges, p)) return penetration_from_nearest_edge(*it);
            }
        }
        template<class SupportFunctionA, class SupportFunctionB> auto minkowski_difference(SupportFunctionA support_a, SupportFunctionB support_b)
        {
            return [=](const float2 & d) { const float2 a = support_a(d); return detail::point{a-support_b(-d), a}; };
        }
    }

    template<class SupportFunctionA, class SupportFunctionB> bool check_intersection(SupportFunctionA support_a, SupportFunctionB support_b, float2 initial_direction)
    {
        return detail::find_intersection_simplex(detail::minkowski_difference(support_a, support_b), initial_direction);
    }

    template<class SupportFunctionA, class SupportFunctionB> std::optional<penetration> find_intersection(SupportFunctionA support_a, SupportFunctionB support_b, float2 initial_direction, float epsilon) 
    { 
        return detail::find_intersection(detail::minkowski_difference(support_a, support_b), initial_direction, epsilon);
    }
}