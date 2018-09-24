// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org/>
#include "collision.h"

namespace collision::detail
{
    static float2 cross(float a, float2 b) { return cross(float3{0,0,a},float3{b,0}).xy(); }
    static float2 cross(float2 a, float b) { return cross(float3{a,0},float3{0,0,b}).xy(); }

    static std::tuple<simplex,float2> make_simplex_point(const point & a) { return {{{a},1}, -a.p}; }
    static std::tuple<simplex,float2> make_simplex_edge(const point & a, const point & b) { return {{{a,b},2}, cross(cross(a.p, b.p-a.p), b.p-a.p)}; }
    static std::tuple<simplex,float2> next_simplex_2(const point & a, const point & b)
    {
        if(dot(b.p-a.p, a.p) < 0) return make_simplex_edge(a,b); // Closest to edge AB
        return make_simplex_point(a); // Closest to point A
    }
    static std::tuple<simplex,float2> next_simplex_3(const point & a, const point & b, const point & c)
    {
        const float2 ab = b.p - a.p, ac = c.p - a.p;
        const float abc = cross(ab,ac);
        if(dot(cross(abc, ac), a.p) < 0)
        {
            if(dot(ac, a.p) < 0) return make_simplex_edge(a,c); // Closest to edge AC
        }
        else if(dot(cross(ab, abc), a.p) >= 0) return {{{a,b,c},3}, {0,0}}; // Inside triangle ABC
        return next_simplex_2(a, b);
    }

    // Expands a simplex by adding a point, and then returns the nearest sub-simplex to the origin and a search direction
    std::tuple<simplex,float2> next_simplex(const simplex & s, const point & a)
    {
        if(s.count == 1) return next_simplex_2(a, s.points[0]);
        if(s.count == 2) return next_simplex_3(a, s.points[0], s.points[1]);
        std::terminate();
    }

    static polytope_edge make_polytope_edge(const point & v0, const point & v1)
    {
        auto n = normalize(cross(v1.p - v0.p, 1));
        return {v0, v1, n, dot(v0.p,n)};
    }

    // Convert a simplex to a polytope, enforcing consistent winding
    std::vector<polytope_edge> make_polytope(const simplex & s)
    {
        const point & a = s.points[0], & b = s.points[1], & c = s.points[2];
        if(cross(b.p-a.p,c.p-a.p) > 0) return {make_polytope_edge(a,b), make_polytope_edge(b,c), make_polytope_edge(c,a)};
        return {make_polytope_edge(a,c), make_polytope_edge(c,b), make_polytope_edge(b,a)};
    }

    // Returns true if polytope was expanded to include point, false if point was already inside polytope
    bool expand_polytope(std::vector<polytope_edge> & edges, point point)
    {
        // If point is inside polytope, we are done, otherwise remove edges which face the point
        auto it = std::remove_if(begin(edges), end(edges), [point](const polytope_edge & edge) { return dot(edge.normal, point.p) > edge.distance; });
        if(it == end(edges)) return false; // No edges removed, point is inside polytope
        if(it == begin(edges)) return false; // ALL edges would be removed, polytope is degenerate and point is colinear with all edges
        edges.erase(it, end(edges));

        // There is now a break somewhere in the list, find it and fill it with two new edges, including our new point
        edges.push_back(edges.front());
        for(size_t i=1; i<edges.size(); ++i)
        {
            if(edges[i-1].v1.p != edges[i].v0.p)
            {
                edges.insert(edges.begin() + i, {make_polytope_edge(edges[i-1].v1, point), make_polytope_edge(point, edges[i].v0)});
                break;
            }
        }
        edges.pop_back();  
        return true;
    }

    // Returns penetration data using the nearest edge to the origin from a fully expanded polytope
    penetration penetration_from_nearest_edge(const polytope_edge & edge)
    {
        const float2 p = edge.normal * edge.distance, ab = edge.v1.p - edge.v0.p;
        const float t = dot(p - edge.v0.p, ab) / dot(ab,ab);
        return {lerp(edge.v0.point_on_a, edge.v1.point_on_a, t), edge.normal, edge.distance};
    }
}
