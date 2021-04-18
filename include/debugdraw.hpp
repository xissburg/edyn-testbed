#ifndef EDYN_TESTBED_DEBUG_DRAW_HPP
#define EDYN_TESTBED_DEBUG_DRAW_HPP

#include <edyn/edyn.hpp>
#include <common/debugdraw/debugdraw.h>
#include <edyn/shapes/polyhedron_shape.hpp>

inline void draw(DebugDrawEncoder &dde, const edyn::sphere_shape &sh) {
    Sphere sphere;
    sphere.center = {0,0,0};
    sphere.radius = sh.radius;
    dde.draw(sphere);
}

inline void draw(DebugDrawEncoder &dde, const edyn::plane_shape &sh) {
    auto center = sh.normal * sh.constant;
    dde.drawQuad(bx::Vec3(sh.normal.x, sh.normal.y, sh.normal.z), bx::Vec3(center.x, center.y, center.z), 10);
}

inline void draw(DebugDrawEncoder &dde, const edyn::cylinder_shape &sh) {
    dde.drawCylinder({-sh.half_length, 0, 0}, {sh.half_length, 0, 0}, sh.radius);
}

inline void draw(DebugDrawEncoder &dde, const edyn::capsule_shape &sh) {
    dde.drawCapsule({-sh.half_length, 0, 0}, {sh.half_length, 0, 0}, sh.radius);
}

inline void draw(DebugDrawEncoder &dde, const edyn::box_shape &sh) {
    auto aabb = Aabb{bx::Vec3(-sh.half_extents.x, -sh.half_extents.y, -sh.half_extents.z), 
                     bx::Vec3( sh.half_extents.x,  sh.half_extents.y,  sh.half_extents.z)};
    dde.draw(aabb);
}

void draw(DebugDrawEncoder &dde, const edyn::mesh_shape &sh);
void draw(DebugDrawEncoder &dde, const edyn::paged_mesh_shape &sh);
void draw(DebugDrawEncoder &dde, const edyn::polyhedron_shape &sh);
void draw(DebugDrawEncoder &dde, const edyn::compound_shape &sh);

void draw(DebugDrawEncoder &dde, entt::entity, const edyn::contact_constraint &, const entt::registry &);

inline void draw(DebugDrawEncoder &dde, entt::entity, const edyn::point_constraint &, const entt::registry &) {

}

void draw(DebugDrawEncoder &dde, entt::entity, const edyn::distance_constraint &, const entt::registry &);
void draw(DebugDrawEncoder &dde, entt::entity, const edyn::soft_distance_constraint &, const entt::registry &);

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::hinge_constraint &con, const entt::registry &reg);

inline
void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::generic_constraint &con, const entt::registry &reg) {

}

#endif // EDYN_TESTBED_DEBUG_DRAW_HPP