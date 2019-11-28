#ifndef EDYN_TESTBED_DEBUG_DRAW_HPP
#define EDYN_TESTBED_DEBUG_DRAW_HPP

#include <edyn/edyn.hpp>
#include <common/debugdraw/debugdraw.h>

void draw(DebugDrawEncoder &dde, const edyn::sphere_shape &sh) {
    Sphere sphere;
    sphere.center = {0,0,0};
    sphere.radius = sh.radius;
    dde.draw(sphere);
}

#endif // EDYN_TESTBED_DEBUG_DRAW_HPP