#include <edyn/math/vector3.hpp>
#include <edyn/math/quaternion.hpp>
#include <bx/math.h>

inline bx::Vec3 to_bx(edyn::vector3 v) {
    return {float(v.x), float(v.y), float(v.z)};
}

inline bx::Quaternion to_bx(edyn::quaternion q) {
    return {float(q.x), float(q.y), float(q.z), float(q.w)};
}
