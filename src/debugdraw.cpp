#include "debugdraw.hpp"
#include <common/debugdraw/debugdraw.h>

void draw(DebugDrawEncoder &dde, const edyn::mesh_shape &sh) {
    dde.setWireframe(false);
    dde.push();

    auto &trimesh = sh.trimesh;

    for (size_t i = 0; i < trimesh->num_edges(); ++i) {
        dde.setColor(trimesh->is_boundary_edge(i) ? 0xff1081ea : 0xffc0c0c0);
        auto vertices = trimesh->get_edge_vertices(i);
        auto &v0 = vertices[0];
        auto &v1 = vertices[1];
        dde.moveTo(v0.x, v0.y, v0.z);
        dde.lineTo(v1.x, v1.y, v1.z);
    }

    dde.pop();
}

void draw(DebugDrawEncoder &dde, const edyn::paged_mesh_shape &sh) {
    dde.setWireframe(false);
    sh.trimesh->visit_all_cached_edges([&] (auto mesh_idx, auto edge_idx) {
        auto trimesh = sh.trimesh->get_submesh(mesh_idx);
        dde.setColor(trimesh->is_boundary_edge(edge_idx) ? 0xff1081ea : 0xffc0c0c0);
        auto vertices = trimesh->get_edge_vertices(edge_idx);
        auto &v0 = vertices[0];
        auto &v1 = vertices[1];
        dde.moveTo(v0.x, v0.y, v0.z);
        dde.lineTo(v1.x, v1.y, v1.z);
    });
}

void draw(DebugDrawEncoder &dde, const edyn::polyhedron_shape &sh) {
    for (size_t i = 0; i < sh.mesh->num_faces(); ++i) {
        auto first = sh.mesh->faces[i * 2];
        auto count = sh.mesh->faces[i * 2 + 1];

        auto i0 = sh.mesh->indices[first];
        auto &v0 = sh.mesh->vertices[i0];

        for (size_t j = 1; j < size_t(count - 1); ++j) {
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
        }, node.shape_var);

        dde.popTransform();
    }
}

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::contact_constraint &con, const entt::registry &reg) {
    //auto &posA = reg.get<edyn::position>(con.body[0]);
    //auto &ornA = reg.get<edyn::orientation>(con.body[0]);
    auto posB = edyn::get_rigidbody_origin(reg, con.body[1]);
    auto ornB = reg.get<edyn::orientation>(con.body[1]);

    auto &cp = reg.get<edyn::contact_point>(entity);
    auto pB = edyn::to_world_space(cp.pivotB, posB, ornB);
    auto tip = pB + cp.normal * 0.2;

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
        posA = edyn::get_rigidbody_present_origin(reg, con.body[0]);
    } else {
        posA = edyn::get_rigidbody_origin(reg, con.body[0]);
    }

    if (reg.has<edyn::present_orientation>(con.body[0])) {
        ornA = reg.get<edyn::present_orientation>(con.body[0]);
    } else {
        ornA = reg.get<edyn::orientation>(con.body[0]);
    }

    if (reg.has<edyn::present_position>(con.body[1])) {
        posB = edyn::get_rigidbody_present_origin(reg, con.body[1]);
    } else {
        posB = edyn::get_rigidbody_origin(reg, con.body[1]);
    }

    if (reg.has<edyn::present_orientation>(con.body[1])) {
        ornB = reg.get<edyn::present_orientation>(con.body[1]);
    } else {
        ornB = reg.get<edyn::orientation>(con.body[1]);
    }

    auto pA = edyn::to_world_space(con.pivot[0], posA, ornA);
    auto pB = edyn::to_world_space(con.pivot[1], posB, ornB);

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
        posA = edyn::get_rigidbody_present_origin(reg, con.body[0]);
    } else {
        posA = edyn::get_rigidbody_origin(reg, con.body[0]);
    }

    if (reg.has<edyn::present_orientation>(con.body[0])) {
        ornA = reg.get<edyn::present_orientation>(con.body[0]);
    } else {
        ornA = reg.get<edyn::orientation>(con.body[0]);
    }

    if (reg.has<edyn::present_position>(con.body[1])) {
        posB = edyn::get_rigidbody_present_origin(reg, con.body[1]);
    } else {
        posB = edyn::get_rigidbody_origin(reg, con.body[1]);
    }

    if (reg.has<edyn::present_orientation>(con.body[1])) {
        ornB = reg.get<edyn::present_orientation>(con.body[1]);
    } else {
        ornB = reg.get<edyn::orientation>(con.body[1]);
    }

    auto pA = edyn::to_world_space(con.pivot[0], posA, ornA);
    auto pB = edyn::to_world_space(con.pivot[1], posB, ornB);

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
        posA = edyn::get_rigidbody_present_origin(reg, con.body[0]);
    } else {
        posA = edyn::get_rigidbody_origin(reg, con.body[0]);
    }

    if (reg.has<edyn::present_orientation>(con.body[0])) {
        ornA = reg.get<edyn::present_orientation>(con.body[0]);
    } else {
        ornA = reg.get<edyn::orientation>(con.body[0]);
    }

    if (reg.has<edyn::present_position>(con.body[1])) {
        posB = edyn::get_rigidbody_present_origin(reg, con.body[1]);
    } else {
        posB = edyn::get_rigidbody_origin(reg, con.body[1]);
    }

    if (reg.has<edyn::present_orientation>(con.body[1])) {
        ornB = reg.get<edyn::present_orientation>(con.body[1]);
    } else {
        ornB = reg.get<edyn::orientation>(con.body[1]);
    }

    auto pA = edyn::to_world_space(con.pivot[0], posA, ornA);
    auto pB = edyn::to_world_space(con.pivot[1], posB, ornB);
    auto axisA = edyn::rotate(ornA, con.axis[0]);
    auto axisB = edyn::rotate(ornB, con.axis[1]);

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
