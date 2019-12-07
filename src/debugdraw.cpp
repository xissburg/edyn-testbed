#include "debugdraw.hpp"

void draw(DebugDrawEncoder &dde, const edyn::contact_constraint &con, const edyn::relation &rel, const entt::registry &reg) {
    auto &posA = reg.get<edyn::position>(rel.entity[0]);
    auto &ornA = reg.get<edyn::orientation>(rel.entity[0]);
    auto &posB = reg.get<edyn::position>(rel.entity[1]);
    auto &ornB = reg.get<edyn::orientation>(rel.entity[1]);

    for (size_t i = 0; i < con.manifold.num_points; ++i) {
        auto &cp = con.manifold.point[i];
        auto pA = posA + edyn::rotate(ornA, cp.pivotA);
        auto pB = posB + edyn::rotate(ornB, cp.pivotB);
        auto normal = edyn::rotate(ornB, cp.normalB);
        auto tip = pB + normal;

        dde.push();

        dde.setColor(0xfffe0000);
        dde.moveTo(pB.x, pB.y, pB.z);
        dde.lineTo(tip.x, tip.y, tip.z);

        dde.pop();
    }
}

void draw(DebugDrawEncoder &dde, const edyn::distance_constraint &con, const edyn::relation &rel, const entt::registry &reg) {
    edyn::vector3 posA, posB;
    edyn::quaternion ornA, ornB;

    if (reg.has<edyn::present_position>(rel.entity[0])) {
        posA = reg.get<edyn::present_position>(rel.entity[0]);
    } else {
        posA = reg.get<edyn::position>(rel.entity[0]);
    }

    if (reg.has<edyn::present_orientation>(rel.entity[0])) {
        ornA = reg.get<edyn::present_orientation>(rel.entity[0]);
    } else {
        ornA = reg.get<edyn::orientation>(rel.entity[0]);
    }

    if (reg.has<edyn::present_position>(rel.entity[1])) {
        posB = reg.get<edyn::present_position>(rel.entity[1]);
    } else {
        posB = reg.get<edyn::position>(rel.entity[1]);
    }

    if (reg.has<edyn::present_orientation>(rel.entity[1])) {
        ornB = reg.get<edyn::present_orientation>(rel.entity[1]);
    } else {
        ornB = reg.get<edyn::orientation>(rel.entity[1]);
    }

    auto pA = posA + edyn::rotate(ornA, con.pivot[0]);
    auto pB = posB + edyn::rotate(ornB, con.pivot[1]);

    dde.push();

    dde.setColor(0xff0000fe);
    dde.moveTo(pA.x, pA.y, pA.z);
    dde.lineTo(pB.x, pB.y, pB.z);

    dde.pop();
}