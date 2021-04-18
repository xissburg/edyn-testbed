#include "debugdraw.hpp"
#include <common/debugdraw/debugdraw.h>
#include <edyn/shapes/compound_shape.hpp>
#include <edyn/shapes/polyhedron_shape.hpp>

void draw(DebugDrawEncoder &dde, const edyn::mesh_shape &sh) {
    dde.setWireframe(false);
    sh.trimesh->visit_all([&] (auto, const edyn::triangle_vertices &vertices) {
        auto &v0 = vertices[0];
        auto &v1 = vertices[1];
        auto &v2 = vertices[2];
        dde.moveTo(v0.x, v0.y, v0.z);
        dde.lineTo(v1.x, v1.y, v1.z);
        dde.lineTo(v2.x, v2.y, v2.z);
        dde.lineTo(v0.x, v0.y, v0.z);
    });
}

void draw(DebugDrawEncoder &dde, const edyn::paged_mesh_shape &sh) {
    dde.setWireframe(false);
    sh.trimesh->visit_cache_all([&] (auto, auto, const edyn::triangle_vertices &vertices) {
        auto &v0 = vertices[0];
        auto &v1 = vertices[1];
        auto &v2 = vertices[2];
        dde.moveTo(v0.x, v0.y, v0.z);
        dde.lineTo(v1.x, v1.y, v1.z);
        dde.lineTo(v2.x, v2.y, v2.z);
        dde.lineTo(v0.x, v0.y, v0.z);
    });
}

void draw(DebugDrawEncoder &dde, const edyn::polyhedron_shape &sh) {
    for (size_t i = 0; i < sh.mesh->num_faces(); ++i) {
        auto first = sh.mesh->faces[i * 2];
        auto count = sh.mesh->faces[i * 2 + 1];

        auto i0 = sh.mesh->indices[first];
        auto &v0 = sh.mesh->vertices[i0];

        for (size_t j = 1; j < count - 1; ++j) {
            auto i1 = sh.mesh->indices[first + j];
            auto i2 = sh.mesh->indices[first + j + 1];
            auto &v1 = sh.mesh->vertices[i1];
            auto &v2 = sh.mesh->vertices[i2];

            auto tri = Triangle { 
                bx::Vec3(v0.x, v0.y, v0.z),
                bx::Vec3(v1.x, v1.y, v1.z),
                bx::Vec3(v2.x, v2.y, v2.z)
            };
            dde.draw(tri);
        }
    }
}

void draw(DebugDrawEncoder &dde, const edyn::compound_shape &sh) {
    for (auto &node : sh.nodes) {
        auto pos = node.position;
        auto orn = node.orientation;

        auto bxquat = bx::Quaternion{float(orn.x), float(orn.y), float(orn.z), float(orn.w)};
        float rot[16];
        bx::mtxQuat(rot, bxquat);
        float rotT[16];
        bx::mtxTranspose(rotT, rot);
        float trans[16];
        bx::mtxTranslate(trans, pos.x, pos.y, pos.z);

        float mtx[16];
        bx::mtxMul(mtx, rotT, trans);

        dde.pushTransform(mtx);
        
        std::visit([&] (auto &&s) {
            draw(dde, s);
        }, node.var);

        dde.popTransform();
    }
}

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::contact_constraint &con, const entt::registry &reg) {
    //auto &posA = reg.get<edyn::position>(con.body[0]);
    //auto &ornA = reg.get<edyn::orientation>(con.body[0]);
    auto &posB = reg.get<edyn::position>(con.body[1]);
    auto &ornB = reg.get<edyn::orientation>(con.body[1]);
    
    auto &cp = reg.get<edyn::contact_point>(entity);
    auto pB = posB + edyn::rotate(ornB, cp.pivotB);
    auto normal = edyn::rotate(ornB, cp.normalB);
    auto tip = pB + normal * 0.3;

    dde.push();

    dde.setColor(0xff3300fe);
    dde.moveTo(pB.x, pB.y, pB.z);
    dde.lineTo(tip.x, tip.y, tip.z);

    dde.pop();
}

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::distance_constraint &con, const entt::registry &reg) {
    edyn::vector3 posA, posB;
    edyn::quaternion ornA, ornB;

    if (reg.has<edyn::present_position>(con.body[0])) {
        posA = reg.get<edyn::present_position>(con.body[0]);
    } else {
        posA = reg.get<edyn::position>(con.body[0]);
    }

    if (reg.has<edyn::present_orientation>(con.body[0])) {
        ornA = reg.get<edyn::present_orientation>(con.body[0]);
    } else {
        ornA = reg.get<edyn::orientation>(con.body[0]);
    }

    if (reg.has<edyn::present_position>(con.body[1])) {
        posB = reg.get<edyn::present_position>(con.body[1]);
    } else {
        posB = reg.get<edyn::position>(con.body[1]);
    }

    if (reg.has<edyn::present_orientation>(con.body[1])) {
        ornB = reg.get<edyn::present_orientation>(con.body[1]);
    } else {
        ornB = reg.get<edyn::orientation>(con.body[1]);
    }

    auto pA = posA + edyn::rotate(ornA, con.pivot[0]);
    auto pB = posB + edyn::rotate(ornB, con.pivot[1]);

    dde.push();

    dde.setColor(0xff0000fe);
    dde.moveTo(pA.x, pA.y, pA.z);
    dde.lineTo(pB.x, pB.y, pB.z);

    dde.pop();
}

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::soft_distance_constraint &con, const entt::registry &reg) {
    edyn::vector3 posA, posB;
    edyn::quaternion ornA, ornB;

    if (reg.has<edyn::present_position>(con.body[0])) {
        posA = reg.get<edyn::present_position>(con.body[0]);
    } else {
        posA = reg.get<edyn::position>(con.body[0]);
    }

    if (reg.has<edyn::present_orientation>(con.body[0])) {
        ornA = reg.get<edyn::present_orientation>(con.body[0]);
    } else {
        ornA = reg.get<edyn::orientation>(con.body[0]);
    }

    if (reg.has<edyn::present_position>(con.body[1])) {
        posB = reg.get<edyn::present_position>(con.body[1]);
    } else {
        posB = reg.get<edyn::position>(con.body[1]);
    }

    if (reg.has<edyn::present_orientation>(con.body[1])) {
        ornB = reg.get<edyn::present_orientation>(con.body[1]);
    } else {
        ornB = reg.get<edyn::orientation>(con.body[1]);
    }

    auto pA = posA + edyn::rotate(ornA, con.pivot[0]);
    auto pB = posB + edyn::rotate(ornB, con.pivot[1]);

    dde.push();

    dde.setColor(0xff0000fe);
    dde.moveTo(pA.x, pA.y, pA.z);
    dde.lineTo(pB.x, pB.y, pB.z);

    dde.pop();
}

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::hinge_constraint &con, const entt::registry &reg) {
    edyn::vector3 posA, posB;
    edyn::quaternion ornA, ornB;

    if (reg.has<edyn::present_position>(con.body[0])) {
        posA = reg.get<edyn::present_position>(con.body[0]);
    } else {
        posA = reg.get<edyn::position>(con.body[0]);
    }

    if (reg.has<edyn::present_orientation>(con.body[0])) {
        ornA = reg.get<edyn::present_orientation>(con.body[0]);
    } else {
        ornA = reg.get<edyn::orientation>(con.body[0]);
    }

    if (reg.has<edyn::present_position>(con.body[1])) {
        posB = reg.get<edyn::present_position>(con.body[1]);
    } else {
        posB = reg.get<edyn::position>(con.body[1]);
    }

    if (reg.has<edyn::present_orientation>(con.body[1])) {
        ornB = reg.get<edyn::present_orientation>(con.body[1]);
    } else {
        ornB = reg.get<edyn::orientation>(con.body[1]);
    }

    auto pA = posA + edyn::rotate(ornA, con.pivot[0]);
    auto pB = posB + edyn::rotate(ornB, con.pivot[1]);
    auto axisA = edyn::rotate(ornA, con.frame[0].column(2));
    auto axisB = edyn::rotate(ornB, con.frame[1].column(2));

    auto pA1 = pA + axisA * 0.2;
    auto pB1 = pB + axisB * 0.2;

    dde.push();

    dde.setColor(0xff0000fe);
    
    dde.moveTo(pA.x, pA.y, pA.z);
    dde.lineTo(pA1.x, pA1.y, pA1.z);
    
    dde.moveTo(pB.x, pB.y, pB.z);
    dde.lineTo(pB1.x, pB1.y, pB1.z);

    dde.pop();
}