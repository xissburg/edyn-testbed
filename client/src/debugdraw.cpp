#include "debugdraw.hpp"
#include "drawing_properties.hpp"
#include <bx/math.h>
#include <common/debugdraw/debugdraw.h>

void assignVertexFrictionColor(DebugDrawEncoder &dde, edyn::triangle_mesh &trimesh, size_t edge_idx, size_t v_idx) {
    auto friction = trimesh.get_vertex_friction(trimesh.get_edge_vertex_indices(edge_idx)[v_idx]);
    auto b = static_cast<uint32_t>(edyn::lerp(0xc0, 0x00, friction));
    auto g = static_cast<uint32_t>(edyn::lerp(0xc0, 0x00, friction));
    auto r = static_cast<uint32_t>(edyn::lerp(0xc0, 0xff, friction));
    dde.setColor(0xff000000 | (b << 16) | (g << 8) | r);
}

void draw(DebugDrawEncoder &dde, const edyn::mesh_shape &sh) {
    dde.setWireframe(false);
    dde.push();

    auto &trimesh = sh.trimesh;

    for (size_t i = 0; i < trimesh->num_edges(); ++i) {
        dde.setColor(trimesh->is_boundary_edge(i) ? 0xff1081ea : 0xffc0c0c0);
        auto vertices = trimesh->get_edge_vertices(i);
        auto &v0 = vertices[0];
        auto &v1 = vertices[1];

        if (trimesh->has_per_vertex_friction()) {
            assignVertexFrictionColor(dde, *trimesh, i, 0);
        }
        dde.moveTo(v0.x, v0.y, v0.z);

        if (trimesh->has_per_vertex_friction()) {
            assignVertexFrictionColor(dde, *trimesh, i, 1);
        }
        dde.lineTo(v1.x, v1.y, v1.z);
    }

    dde.pop();
}

void draw(DebugDrawEncoder &dde, const edyn::paged_mesh_shape &sh) {
    dde.setWireframe(false);
    sh.trimesh->visit_all_cached_edges([&](auto mesh_idx, auto edge_idx) {
        auto trimesh = sh.trimesh->get_submesh(mesh_idx);
        dde.setColor(trimesh->is_boundary_edge(edge_idx) ? 0xff1081ea : 0xffc0c0c0);
        auto vertices = trimesh->get_edge_vertices(edge_idx);
        auto &v0 = vertices[0];
        auto &v1 = vertices[1];

        if (trimesh->has_per_vertex_friction()) {
            assignVertexFrictionColor(dde, *trimesh, edge_idx, 0);
        }
        dde.moveTo(v0.x, v0.y, v0.z);


        if (trimesh->has_per_vertex_friction()) {
            assignVertexFrictionColor(dde, *trimesh, edge_idx, 1);
        }
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

        auto bxquat = to_bx(orn);
        float rot[16];
        bx::mtxQuat(rot, bxquat);
        float rotT[16];
        bx::mtxTranspose(rotT, rot);
        float trans[16];
        bx::mtxTranslate(trans, pos.x, pos.y, pos.z);

        float mtx[16];
        bx::mtxMul(mtx, rotT, trans);

        dde.pushTransform(mtx);

        std::visit([&](auto &&s) {
            draw(dde, s);
        }, node.shape_var);

        dde.popTransform();
    }
}

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::contact_manifold &manifold, const entt::registry &reg) {
    //auto &posA = reg.get<edyn::position>(con.body[0]);
    //auto &ornA = reg.get<edyn::orientation>(con.body[0]);
    auto posB = edyn::get_rigidbody_origin(reg, manifold.body[1]);
    auto ornB = reg.get<edyn::orientation>(manifold.body[1]);

    manifold.each_point([&](auto &cp) {
        auto pB = edyn::to_world_space(cp.pivotB, posB, ornB);
        auto tip = pB + cp.normal * 0.1;

        dde.push();

        dde.setColor(0xff3300fe);
        dde.moveTo(pB.x, pB.y, pB.z);
        dde.lineTo(tip.x, tip.y, tip.z);

        dde.pop();
    });
}

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::contact_constraint &con, const entt::registry &reg) {
    auto &manifold = reg.get<edyn::contact_manifold>(entity);
    draw(dde, entity, manifold, reg);
}

auto get_transforms(const entt::registry &reg, const edyn::constraint_base &con) {
    auto posA = reg.any_of<edyn::present_position>(con.body[0]) ?
        edyn::get_rigidbody_present_origin(reg, con.body[0]) :
        edyn::get_rigidbody_origin(reg, con.body[0]);

    auto ornA = reg.any_of<edyn::present_orientation>(con.body[0]) ?
        static_cast<edyn::quaternion>(reg.get<edyn::present_orientation>(con.body[0])) :
        static_cast<edyn::quaternion>(reg.get<edyn::orientation>(con.body[0]));

    auto posB = reg.any_of<edyn::present_position>(con.body[1]) ?
        edyn::get_rigidbody_present_origin(reg, con.body[1]) :
        edyn::get_rigidbody_origin(reg, con.body[1]);

    auto ornB = reg.any_of<edyn::present_orientation>(con.body[1]) ?
        static_cast<edyn::quaternion>(reg.get<edyn::present_orientation>(con.body[1])) :
        static_cast<edyn::quaternion>(reg.get<edyn::orientation>(con.body[1]));

    return std::make_tuple(posA, ornA, posB, ornB);
}

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::distance_constraint &con, const entt::registry &reg) {
    auto [posA, ornA, posB, ornB] = get_transforms(reg, con);

    auto pA = edyn::to_world_space(con.pivot[0], posA, ornA);
    auto pB = edyn::to_world_space(con.pivot[1], posB, ornB);

    dde.push();

    dde.setColor(0xff0000fe);
    dde.moveTo(pA.x, pA.y, pA.z);
    dde.lineTo(pB.x, pB.y, pB.z);

    dde.pop();
}

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::soft_distance_constraint &con, const entt::registry &reg) {
    auto [posA, ornA, posB, ornB] = get_transforms(reg, con);

    auto pA = edyn::to_world_space(con.pivot[0], posA, ornA);
    auto pB = edyn::to_world_space(con.pivot[1], posB, ornB);

    dde.push();

    dde.setColor(0xff0000fe);
    dde.moveTo(pA.x, pA.y, pA.z);
    dde.lineTo(pB.x, pB.y, pB.z);

    dde.pop();
}

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::hinge_constraint &con, const entt::registry &reg) {
    auto [posA, ornA, posB, ornB] = get_transforms(reg, con);

    auto pA = edyn::to_world_space(con.pivot[0], posA, ornA);
    auto pB = edyn::to_world_space(con.pivot[1], posB, ornB);
    auto axisA = edyn::rotate(ornA, con.frame[0].column(0));
    auto axisB = edyn::rotate(ornB, con.frame[1].column(0));

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

void draw(DebugDrawEncoder &dde, entt::entity entity, const edyn::cone_constraint &con, const entt::registry &reg) {
    auto *properties = reg.try_get<DrawingProperties>(entity);

    if (!properties) {
        return;
    }

    auto [posA, ornA, posB, ornB] = get_transforms(reg, con);

    float rot[16];
    float rotT[16];
    float trans[16];
    float mtx[16];

    bx::mtxQuat(rot, to_bx(ornA));
    bx::mtxTranspose(rotT, rot);
    bx::mtxTranslate(trans, posA.x, posA.y, posA.z);
    bx::mtxMul(mtx, rotT, trans);

    dde.pushTransform(mtx);

    float frame[16] = {(float)con.frame.row[0].x, (float)con.frame.row[0].y, (float)con.frame.row[0].z, 0.f,
                       (float)con.frame.row[1].x, (float)con.frame.row[1].y, (float)con.frame.row[1].z, 0.f,
                       (float)con.frame.row[2].x, (float)con.frame.row[2].y, (float)con.frame.row[2].z, 0.f,
                       0.f, 0.f, 0.f, 1.f};
    bx::mtxTranspose(rotT, frame);
    bx::mtxTranslate(trans, con.pivot[0].x, con.pivot[0].y, con.pivot[0].z);
    bx::mtxMul(mtx, rotT, trans);

    dde.pushTransform(mtx);

    auto radius0 = std::sin(std::atan(con.span_tan[0]));
    auto radius1 = std::sin(std::atan(con.span_tan[1]));

    auto num_points = 36;

    for (auto i = 0; i < num_points + 1; ++i) {
        auto angle = (float)i / (float)num_points * edyn::pi2;
        auto cos = std::cos(angle);
        auto sin = std::sin(angle);
        auto p = edyn::vector3{};
        p.y = cos * radius0;
        p.z = sin * radius1;
        p.x = std::sqrt(1 - (p.y * p.y + p.z * p.z));
        p *= properties->scale;

        if (i == 0) {
            dde.moveTo(to_bx(p));
        } else {
            dde.lineTo(to_bx(p));
        }

        if (i % (num_points / 8) == 0) {
            dde.moveTo(0,0,0);
            dde.lineTo(to_bx(p));
        }
    }

    dde.popTransform();
    dde.popTransform();
}
