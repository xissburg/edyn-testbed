#ifndef EDYN_TESTBED_DEBUG_DRAW_HPP
#define EDYN_TESTBED_DEBUG_DRAW_HPP

#include <edyn/edyn.hpp>
#include <common/debugdraw/debugdraw.h>
#include <edyn/shapes/polyhedron_shape.hpp>
#include "bx_util.hpp"

inline void draw(DebugDrawEncoder &dde, const edyn::sphere_shape &sh) {
    Sphere sphere;
    sphere.center = {0,0,0};
    sphere.radius = sh.radius;
    dde.draw(sphere);
}

inline void draw(DebugDrawEncoder &dde, const edyn::plane_shape &sh) {
    auto center = sh.normal * sh.constant;
    dde.drawQuad(to_bx(-sh.normal), to_bx(center), 20);
}

inline void draw(DebugDrawEncoder &dde, const edyn::cylinder_shape &sh) {
    dde.drawCylinder({-sh.half_length, 0, 0}, {sh.half_length, 0, 0}, sh.radius);
}

inline void draw(DebugDrawEncoder &dde, const edyn::capsule_shape &sh) {
    dde.drawCapsule({-sh.half_length, 0, 0}, {sh.half_length, 0, 0}, sh.radius);
}

inline void draw(DebugDrawEncoder &dde, const edyn::box_shape &sh) {
    auto aabb = Aabb{to_bx(-sh.half_extents), to_bx(sh.half_extents)};
    dde.draw(aabb);
}

void draw(DebugDrawEncoder &dde, const edyn::mesh_shape &sh);
void draw(DebugDrawEncoder &dde, const edyn::paged_mesh_shape &sh);
void draw(DebugDrawEncoder &dde, const edyn::polyhedron_shape &sh);
void draw(DebugDrawEncoder &dde, const edyn::compound_shape &sh);

void draw(DebugDrawEncoder &dde, entt::entity, const edyn::contact_constraint &, const entt::registry &);

inline void draw(DebugDrawEncoder &dde, entt::entity, const edyn::point_constraint &, const entt::registry &) {

}

inline void draw(DebugDrawEncoder &dde, entt::entity, const edyn::cvjoint_constraint &, const entt::registry &) {

}

void draw(DebugDrawEncoder &dde, entt::entity, const edyn::cone_constraint &, const entt::registry &);

void draw(DebugDrawEncoder &dde, entt::entity, const edyn::distance_constraint &, const entt::registry &);
void draw(DebugDrawEncoder &dde, entt::entity, const edyn::soft_distance_constraint &, const entt::registry &);

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::hinge_constraint &con, const entt::registry &reg);

inline
void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::generic_constraint &con, const entt::registry &reg) {

}

inline
void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::null_constraint &con, const entt::registry &reg) {}

inline
void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::gravity_constraint &con, const entt::registry &reg) {}

#endif // EDYN_TESTBED_DEBUG_DRAW_HPP
