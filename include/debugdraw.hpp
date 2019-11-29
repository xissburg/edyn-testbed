#ifndef EDYN_TESTBED_DEBUG_DRAW_HPP
#define EDYN_TESTBED_DEBUG_DRAW_HPP

#include <edyn/edyn.hpp>
#include <common/debugdraw/debugdraw.h>

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

void draw(DebugDrawEncoder &dde, const edyn::contact_constraint &con, const edyn::relation &rel, const entt::registry &reg);

inline void draw(DebugDrawEncoder &dde, const edyn::point_constraint &con, const edyn::relation &rel, const entt::registry &reg) {

}

void draw(DebugDrawEncoder &dde, const edyn::distance_constraint &con, const edyn::relation &rel, const entt::registry &reg);

#endif // EDYN_TESTBED_DEBUG_DRAW_HPP