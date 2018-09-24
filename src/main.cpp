// This is free and unencumbered software released into the public domain.
// For more information, please refer to <http://unlicense.org/>
#include <iostream>
#include <variant>
#include <GLFW/glfw3.h>
#include "collision.h"
#include "physics.h"

void glVertex(const float2 & v) { glVertex2f(v.x, v.y); }

struct circle { float2 center; float radius; };
struct posed_box { float2 half_extent; float2 position; float orientation; };
struct segment { float2 p0, p1; };
struct convex_polygon { float2 points[6]; };

float2 support(circle c, float2 direction) { return c.center + normalize(direction) * c.radius; }
float2 support(posed_box b, float2 direction) 
{ 
    float2 local_dir = rot(-b.orientation, direction);
    return b.position + rot(b.orientation, float2{local_dir.x > 0 ? b.half_extent.x : -b.half_extent.x, local_dir.y > 0 ? b.half_extent.y : -b.half_extent.y});
}
float2 support(segment l, float2 direction) { return dot(direction, l.p1-l.p0) > 0 ? l.p1 : l.p0; }
float2 support(convex_polygon p, float2 direction) 
{ 
    float2 best = p.points[0];
    float best_d = dot(p.points[0], direction);
    for(int i=1; i<6; ++i)
    {
        float d = dot(p.points[i], direction);
        if(d > best_d)
        {
            best = p.points[i];
            best_d = d;
        }
    }
    return best;
}

void draw(circle c)
{
    glBegin(GL_LINE_LOOP);
    for(int i=0, n=48; i<n; ++i)
    {
        const float a = i*6.28318531f/n;
        glVertex2f(c.center.x + std::cos(a)*c.radius, c.center.y + std::sin(a)*c.radius);
    }
    glEnd();
}
void draw(posed_box b)
{
    glBegin(GL_LINE_LOOP);
    glVertex(b.position + rot(b.orientation, float2{-b.half_extent.x, -b.half_extent.y}));
    glVertex(b.position + rot(b.orientation, float2{+b.half_extent.x, -b.half_extent.y}));
    glVertex(b.position + rot(b.orientation, float2{+b.half_extent.x, +b.half_extent.y}));
    glVertex(b.position + rot(b.orientation, float2{-b.half_extent.x, +b.half_extent.y}));
    glVertex(b.position + rot(b.orientation, float2{-b.half_extent.x, -b.half_extent.y}));
    glEnd();
}
void draw(segment s)
{
    glBegin(GL_LINES);
    glVertex2f(s.p0.x, s.p0.y);
    glVertex2f(s.p1.x, s.p1.y);
    glEnd();
}
void draw(convex_polygon p)
{
    glBegin(GL_LINE_LOOP);
    for(auto & v : p.points) glVertex(v);
    glEnd();
}

template<class T> auto make_support_function(T shape) { return [shape](float2 direction) { return support(shape, direction); }; }

using shape = std::variant<circle, posed_box, segment, convex_polygon>;

std::optional<collision::penetration> find_intersection(const shape & shape_a, const shape & shape_b, const float2 & initial_direction)
{
    return std::visit([initial_direction](const auto & a, const auto & b) 
    { 
        auto func_a = make_support_function(a);
        auto func_b = make_support_function(b);
        return collision::find_intersection(func_a, func_b, {1,0}); 
    }, shape_a, shape_b);
}

struct entity
{
    physics::rigidbody body;
    float radius;
    int type;

    shape get_shape() const 
    {
        switch(type)
        {
        case 0: return circle{body.position, radius};
        case 1: return posed_box{float2{radius}, body.position, body.orientation};
        case 2:
        {
            convex_polygon p;
            for(int i=0; i<6; ++i)
            {
                const float a = i*6.28318531f/6;
                p.points[i] = body.position + rot(body.orientation, float2{std::cos(a)*radius, std::sin(a)*radius});
            }
            return p;
        }
        case 3:
        {
            convex_polygon p;
            for(int i=0; i<6; ++i)
            {
                const float a = i*6.28318531f/3;
                p.points[i] = body.position + rot(body.orientation, float2{std::cos(a)*radius, std::sin(a)*radius});
            }
            return p;
        }
        default: throw std::logic_error("bad type");
        }
    }        
};

template<class ShapeA, class ShapeB> std::optional<collision::penetration> find_intersection(const ShapeA & shape_a, const ShapeB & shape_b)
{
    return collision::find_intersection(make_support_function(shape_a), make_support_function(shape_b));
}

#include <vector>
#include <chrono>
#include <random>
int main() try
{
    const segment segs[] 
    {
        {{0.1f,-0.3f},{0.7f,0.3f}},
        {{-1.5f,0},{0,-1.0f}},
    };

    struct world
    {
        std::mt19937 rng;
        std::vector<entity> entities;
    };
    world w;

    glfwInit();
    auto win = glfwCreateWindow(1280, 720, "Simulation", nullptr, nullptr);
    glfwSetWindowUserPointer(win, &w);
    glfwSetKeyCallback(win, [](GLFWwindow * win, int key, int scancode, int action, int mods)
    {
        auto & w = *reinterpret_cast<world *>(glfwGetWindowUserPointer(win));
        if(action != GLFW_RELEASE)
        {           
            std::normal_distribution<float> radius_dist(0.14f, 0.02f);  
            float radius = std::max(radius_dist(w.rng), 0.05f);
            switch(key)
            {
            case GLFW_KEY_1: w.entities.push_back({{{0.0f,1}, {0,0}, 0.0f, 0.0f, physics::compute_mass_for_circle(1.0f, radius), 0.4f}, radius, 0}); break;
            case GLFW_KEY_2: w.entities.push_back({{{0.0f,1}, {0,0}, 0.0f, 0.0f, physics::compute_mass_for_box(1.0f, float2{radius*2}), 0.4f}, radius, 1}); break;
            case GLFW_KEY_3: w.entities.push_back({{{0.0f,1}, {0,0}, 0.0f, 0.0f, physics::compute_mass_for_circle(1.0f, radius*0.9f), 0.4f}, radius, 2}); break;
            case GLFW_KEY_4: w.entities.push_back({{{0.0f,1}, {0,0}, 0.0f, 0.0f, physics::compute_mass_for_circle(1.0f, radius*0.9f), 0.4f}, radius, 3}); break;
            }            
        }
    });

    glfwMakeContextCurrent(win);
    using clock = std::chrono::high_resolution_clock;
    auto t0 = clock::now();
    while(!glfwWindowShouldClose(win))
    {
        glfwPollEvents();

        const auto t1 = clock::now();
        const auto timestep = std::chrono::duration<float>(t1-t0).count();
        t0 = t1;

        // Add gravity and integrate
        float2 accel {0,-1};
        for(auto & e : w.entities)
        {
            e.body.position += e.body.velocity()*timestep + accel*(timestep*timestep/2);
            e.body.orientation += e.body.spin()*timestep;
            e.body.momentum += accel*(e.body.mass_dist.mass*timestep);
        }

        // Remove rigidbodies that have fallen off the screen
        auto it = std::remove_if(begin(w.entities), end(w.entities), [](const entity & e) { return e.body.position.y < -3; });
        w.entities.erase(it, end(w.entities));

        // Collision detection
        std::vector<physics::linear_constraint> constraints;

        // Collide with each other
        for(auto & a : w.entities)
        {
            for(auto & b : w.entities)
            {
                if(&b <= &a) continue;
                if(auto pen = find_intersection(a.get_shape(), b.get_shape(), b.body.position - a.body.position))
                {
                    float v = dot(b.body.velocity() - a.body.velocity(), pen->normal_a_to_b());
                    float dvel = std::max(v * -std::min(a.body.elasticity, b.body.elasticity), pen->penetration_depth() / 0.1f);
                    constraints.push_back({&a.body, &b.body, pen->point_on_a()-a.body.position, pen->point_on_b()-b.body.position, pen->normal_a_to_b(), dvel, 0, 1000});
                }
            }
        }

        // Collide with world
        for(auto & e : w.entities)
        {
            for(auto seg : segs)
            {
                if(auto pen = find_intersection(e.get_shape(), seg, seg.p0 - e.body.position))
                {
                    float v = dot(-e.body.velocity(), pen->normal_a_to_b());
                    float dvel = std::max(v * -e.body.elasticity, pen->penetration_depth() / 0.1f);
                    constraints.push_back({&e.body, nullptr, pen->point_on_a()-e.body.position, pen->point_on_b(), pen->normal_a_to_b(), dvel, 0, 1000});
                }
            }
        }

        // Run solver
        solve_constraints(constraints);

        // Set up matrices
        {
            int w, h;
            glfwGetFramebufferSize(win, &w, &h);
            float aspect = (float)w/h;
            auto m = linalg::frustum_matrix(-aspect, aspect, -1.0f, 1.0f, 1.0f, 2.0f, linalg::neg_z, linalg::neg_one_to_one);
            glLoadMatrixf(&m.x.x);
            glTranslatef(0,0,-1);
        }

        // Render scene
        glClear(GL_COLOR_BUFFER_BIT);
        for(const auto & e : w.entities) std::visit([](const auto & s) { draw(s); }, e.get_shape());
        for(const auto & seg : segs) draw(seg);        
        glfwSwapBuffers(win);        
    }
    glfwTerminate();
    return EXIT_SUCCESS;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}