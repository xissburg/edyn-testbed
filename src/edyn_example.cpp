#include "edyn_example.hpp"
#include <dear-imgui/imgui.h>
#include <fenv.h>

void cmdTogglePause(const void* _userData) {
    ((EdynExample *)_userData)->togglePausePhysics();
}

void cmdStepSimulation(const void* _userData) {
    ((EdynExample *)_userData)->stepPhysics();
}

void OnCreateIsland(entt::registry &registry, entt::entity entity) {
    registry.emplace<ColorComponent>(entity, 0xff000000 | (0x00ffffff & rand()));
}

void OnDestroyIsland(entt::registry &registry, entt::entity entity) {
    //registry.remove<ColorComponent>(entity);
}

void EdynExample::init(int32_t _argc, const char* const* _argv, uint32_t _width, uint32_t _height)
{
    //feenableexcept(FE_INVALID | FE_OVERFLOW | FE_DIVBYZERO);

    Args args(_argc, _argv);

    m_width  = _width;
    m_height = _height;
    m_debug  = BGFX_DEBUG_NONE;
    m_reset  = BGFX_RESET_VSYNC;

    bgfx::Init init;
    init.type     = args.m_type;
    init.vendorId = args.m_pciId;
    init.resolution.width  = m_width;
    init.resolution.height = m_height;
    init.resolution.reset  = m_reset;
    init.callback = &m_callback;
    bgfx::init(init);

    // Enable debug text.
    bgfx::setDebug(m_debug);

    // Set view 0 clear state.
    bgfx::setViewClear(0
        , BGFX_CLEAR_COLOR|BGFX_CLEAR_DEPTH
        , 0x303030ff
        , 1.0f
        , 0
        );

    // Init DebugDraw.
    ddInit();

    imguiCreate();

    cameraCreate();

    cameraSetPosition({ 0.0f, 2.0f, -12.0f });
    cameraSetVerticalAngle(0.0f);

    m_timestamp = bx::getHPCounter();

    m_registry.reset(new entt::registry);

    m_registry->on_construct<edyn::island>().connect<&OnCreateIsland>();
    m_registry->on_destroy<edyn::island>().connect<&OnDestroyIsland>();

    edyn::init();
    edyn::attach(*m_registry);
    m_fixed_dt_ms = static_cast<int>(edyn::get_fixed_dt(*m_registry) * 1000);
    m_num_velocity_iterations = edyn::get_solver_velocity_iterations(*m_registry);
    m_num_position_iterations = edyn::get_solver_position_iterations(*m_registry);
    m_gui_gravity = m_gravity = -edyn::get_gravity(*m_registry).y;

    // Input bindings
    m_bindings = (InputBinding*)BX_ALLOC(entry::getAllocator(), sizeof(InputBinding)*3);
    m_bindings[0].set(entry::Key::KeyP, entry::Modifier::None, 1, cmdTogglePause,  this);
    m_bindings[1].set(entry::Key::KeyL, entry::Modifier::None, 1, cmdStepSimulation, this);
    m_bindings[2].end();

    inputAddBindings("base", m_bindings);

    m_footer_text = m_default_footer_text;

#ifdef EDYN_SOUND_ENABLED
    m_soloud.init();
#endif

    createScene();
}

int EdynExample::shutdown()
{
    destroyScene();

    // Cleanup.
    ddShutdown();

    imguiDestroy();

    cameraDestroy();

    inputRemoveBindings("base");
	BX_FREE(entry::getAllocator(), m_bindings);

    // Shutdown bgfx.
    bgfx::shutdown();

    edyn::detach(*m_registry);
    edyn::deinit();

#ifdef EDYN_SOUND_ENABLED
    m_soloud.deinit();
#endif

    m_registry.reset();

    return 0;
}

bool EdynExample::update()
{
    if (entry::processEvents(m_width, m_height, m_debug, m_reset, &m_mouseState) ) {
        return false;
    }

    // Set view 0 default viewport.
    bgfx::setViewRect(0, 0, 0, uint16_t(m_width), uint16_t(m_height) );

    bgfx::touch(0);

    int64_t now = bx::getHPCounter();
    const int64_t frameTime = now - m_timestamp;
    m_timestamp = now;
    const double freq = double(bx::getHPFrequency());
    const float deltaTime = float(frameTime/freq);

    cameraUpdate(deltaTime, m_mouseState);

    // Set view and projection matrix for view 0.
    float viewMtx[16];
    cameraGetViewMtx(viewMtx);

    float proj[16];
    bx::mtxProj(proj, 60.0f, float(m_width)/float(m_height), 0.1f, 10000.0f, bgfx::getCaps()->homogeneousDepth);

    bgfx::setViewTransform(0, viewMtx, proj);
    bgfx::setViewRect(0, 0, 0, uint16_t(m_width), uint16_t(m_height) );

#ifdef EDYN_SOUND_ENABLED
    auto camPos = cameraGetPosition();
    m_soloud.set3dListenerPosition(camPos.x, camPos.y, camPos.z);
    m_soloud.update3dAudio();
#endif

    updateGUI();

    updateSettings();

    updatePicking(viewMtx, proj);

    updatePhysics(deltaTime);

    // Draw stuff.
    DebugDrawEncoder dde;
    dde.begin(0);

    // Grid.
    dde.drawGrid(Axis::Y, { 0.0f, 0.0f, 0.0f });

    auto shape_views_tuple = edyn::get_tuple_of_shape_views(*m_registry);

    // Draw dynamic entities.
    {
        auto view = m_registry->view<edyn::shape_index, edyn::present_position, edyn::present_orientation>();
        view.each([&] (auto ent, auto &sh_idx, auto &pos, auto &orn) {
            dde.push();

            uint32_t color = 0xffffffff;

            if (m_registry->any_of<edyn::sleeping_tag>(ent)) {
                color = 0x80000000;
            } else {
                auto *resident = m_registry->try_get<edyn::island_resident>(ent);
                if (resident) {
                    color = m_registry->get<ColorComponent>(resident->island_entity);
                }
            }
            dde.setColor(color);
            //dde.setWireframe(true);

            auto bxquat = bx::Quaternion{float(orn.x), float(orn.y), float(orn.z), float(orn.w)};
            float rot[16];
            bx::mtxQuat(rot, bxquat);

            float rotT[16];
            bx::mtxTranspose(rotT, rot);

            float trans[16];
            auto origin = edyn::get_rigidbody_origin(*m_registry, ent);
            bx::mtxTranslate(trans, origin.x, origin.y, origin.z);

            float mtx[16];
            bx::mtxMul(mtx, rotT, trans);

            dde.pushTransform(mtx);

            edyn::visit_shape(sh_idx, ent, shape_views_tuple, [&] (auto &&s) {
                draw(dde, s);
            });

            dde.drawAxis(0, 0, 0, m_rigid_body_axes_size);

            dde.popTransform();

            dde.pop();
        });
    }

    // Draw AABBs.
    #if 0
    {
        dde.push();

        const uint32_t color = 0xff0000f2;
        dde.setColor(color);
        dde.setWireframe(true);

        auto view = m_registry->view<edyn::AABB>();
        view.each([&] (edyn::AABB &aabb) {
            dde.draw(Aabb{{aabb.min.x, aabb.min.y, aabb.min.z}, {aabb.max.x, aabb.max.y, aabb.max.z}});
        });

        dde.pop();
    }
    #endif

    // Draw static and kinematic entities.
    {
        auto view = m_registry->view<edyn::shape_index, edyn::position, edyn::orientation>();
        view.each([&] (auto ent, auto &sh_idx, auto &pos, auto &orn) {
            if (!m_registry->any_of<edyn::static_tag>(ent) &&
                !m_registry->any_of<edyn::kinematic_tag>(ent)) {
                return;
            }

            dde.push();

            uint32_t color = 0xffa0a0a0;
            dde.setColor(color);
            //dde.setWireframe(true);

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

            edyn::visit_shape(sh_idx, ent, shape_views_tuple, [&] (auto &&s) {
                draw(dde, s);
            });

            dde.drawAxis(0, 0, 0, m_rigid_body_axes_size);
            dde.popTransform();
            dde.pop();
        });
    }

    // Draw amorphous entities.
    {
        auto view = m_registry->view<edyn::present_position, edyn::present_orientation>(entt::exclude_t<edyn::shape_index>{});
        view.each([&] (auto ent, auto &pos, auto &orn) {
            dde.push();

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
            dde.drawAxis(0, 0, 0);
            dde.popTransform();

            dde.pop();
        });
    }

    // Draw constraints.
    {
        std::apply([&] (auto ...c) {
            (
            m_registry->view<decltype(c)>().each([&] (auto ent, auto &con) {
                draw(dde, ent, con, *m_registry);
            }), ...);
        }, edyn::constraints_tuple);
    }

    drawRaycast(dde);

    dde.end();

    // Advance to next frame. Rendering thread will be kicked to
    // process submitted rendering primitives.
    bgfx::frame();

    return true;
}

void drawRaycastResult(DebugDrawEncoder &dde, edyn::box_shape &box,
                       const edyn::shape_raycast_result &result,
                       edyn::vector3 rayPos, edyn::vector3 rayDir,
                       edyn::vector3 pos, edyn::quaternion orn) {
    auto rayPosLocal = edyn::to_object_space(rayPos, pos, orn);
    auto rayDirLocal = edyn::rotate(edyn::conjugate(orn), rayDir);
    auto intersection = rayPosLocal + rayDirLocal * result.fraction;

    auto face_idx = std::get<edyn::box_raycast_info>(result.info_var).face_index;
    auto [feature, feature_idx] =
        box.get_closest_feature_on_face(face_idx, intersection, 0.01);

    switch (feature) {
    case edyn::box_feature::vertex: {
        auto v = box.get_vertex(feature_idx);
        auto normal = edyn::normalize(rayDirLocal);
        dde.drawQuad({normal.x, normal.y, normal.z}, {v.x, v.y, v.z}, 0.015f);
        break;
    } case edyn::box_feature::edge: {
        auto v = box.get_edge(feature_idx);
        dde.moveTo(v[0].x, v[0].y, v[0].z);
        dde.lineTo(v[1].x, v[1].y, v[1].z);
        break;
    } case edyn::box_feature::face: {
        auto v = box.get_face(feature_idx);
        dde.moveTo(v[0].x, v[0].y, v[0].z);
        for (auto i = 0; i < 4; ++i) {
            auto j = (i + 1) % 4;
            dde.lineTo(v[j].x, v[j].y, v[j].z);
        }
    }}
}

void drawRaycastResult(DebugDrawEncoder &dde, edyn::cylinder_shape &cylinder,
                       const edyn::shape_raycast_result &result,
                       edyn::vector3 rayPos, edyn::vector3 rayDir,
                       edyn::vector3 pos, edyn::quaternion orn) {
    auto rayPosLocal = edyn::to_object_space(rayPos, pos, orn);
    auto rayDirLocal = edyn::rotate(edyn::conjugate(orn), rayDir);
    auto intersection = rayPosLocal + rayDirLocal * result.fraction;

    auto info = std::get<edyn::cylinder_raycast_info>(result.info_var);
    edyn::vector3 vertices[] = {
        {cylinder.half_length, 0, 0},
        {-cylinder.half_length, 0, 0},
    };
    auto axis = edyn::vector3_x;
    auto feature = info.feature;
    auto feature_index = info.face_index;
    auto tolerance = edyn::scalar(0.01);

    // Set feature as cap_edge if intersection is close to it.
    if (info.feature == edyn::cylinder_feature::face) {
        if (edyn::distance_sqr(intersection, vertices[info.face_index]) > edyn::square(cylinder.radius - tolerance)) {
            feature = edyn::cylinder_feature::cap_edge;
        }
    } else if (info.feature == edyn::cylinder_feature::side_edge) {
        auto proj = edyn::dot(intersection, axis);
        if (std::abs(proj) > cylinder.half_length - tolerance) {
            feature = edyn::cylinder_feature::cap_edge;
            feature_index = proj > 0 ? 0 : 1;
        }
    }

    switch (feature) {
    case edyn::cylinder_feature::cap_edge: {
        auto center = vertices[feature_index];
        dde.drawCircle({axis.x, axis.y, axis.z},
                       {center.x, center.y, center.z}, cylinder.radius);
        break;
    } case edyn::cylinder_feature::face: {
        auto from = vertices[feature_index];
        auto to = from + axis * 0.001f * (feature_index == 0 ? 1 : -1);
        dde.drawCylinder({from.x, from.y, from.z}, {to.x, to.y, to.z}, cylinder.radius);
        break;
    } case edyn::cylinder_feature::side_edge: {
        auto p0 = edyn::vector3 {vertices[0].x, intersection.y, intersection.z};
        auto p1 = edyn::vector3 {vertices[1].x, intersection.y, intersection.z};
        dde.moveTo(p0.x, p0.y, p0.z);
        dde.lineTo(p1.x, p1.y, p1.z);
    }}
}

void drawRaycastResult(DebugDrawEncoder &dde, edyn::sphere_shape &sphere,
                       const edyn::shape_raycast_result &result,
                       edyn::vector3 rayPos, edyn::vector3 rayDir,
                       edyn::vector3 pos, edyn::quaternion orn) {
    auto axis = edyn::rotate(edyn::conjugate(orn), edyn::normalize(rayDir));
    dde.drawCircle({axis.x, axis.y, axis.z},
                   {0,0,0}, sphere.radius);
}

void drawRaycastResult(DebugDrawEncoder &dde, edyn::capsule_shape &capsule,
                       const edyn::shape_raycast_result &result,
                       edyn::vector3 rayPos, edyn::vector3 rayDir,
                       edyn::vector3 pos, edyn::quaternion orn) {
    auto info = std::get<edyn::capsule_raycast_info>(result.info_var);

    edyn::vector3 vertices[] = {
        {capsule.half_length, 0, 0},
        {-capsule.half_length, 0, 0},
    };
    auto axis = edyn::vector3_x;

    switch (info.feature) {
    case edyn::capsule_feature::hemisphere: {
        auto center = vertices[info.hemisphere_index];
        dde.drawCircle({axis.x, axis.y, axis.z},
                       {center.x, center.y, center.z}, capsule.radius);
        break;
    } case edyn::capsule_feature::side: {
        auto rayPosLocal = edyn::to_object_space(rayPos, pos, orn);
        auto rayDirLocal = edyn::rotate(edyn::conjugate(orn), rayDir);
        auto intersection = rayPosLocal + rayDirLocal * result.fraction;

        auto p0 = edyn::vector3 {vertices[0].x, intersection.y, intersection.z};
        auto p1 = edyn::vector3 {vertices[1].x, intersection.y, intersection.z};
        dde.moveTo(p0.x, p0.y, p0.z);
        dde.lineTo(p1.x, p1.y, p1.z);
    }}
}

void drawRaycastResult(DebugDrawEncoder &dde, edyn::polyhedron_shape &poly,
                       const edyn::shape_raycast_result &result,
                       edyn::vector3 rayPos, edyn::vector3 rayDir,
                       edyn::vector3 pos, edyn::quaternion orn) {
    auto rayPosLocal = edyn::to_object_space(rayPos, pos, orn);
    auto rayDirLocal = edyn::rotate(edyn::conjugate(orn), rayDir);
    auto intersection = rayPosLocal + rayDirLocal * result.fraction;

    auto face_idx = std::get<edyn::polyhedron_raycast_info>(result.info_var).face_index;
    auto tolerance = edyn::scalar(0.01);
    auto tolerance_sqr = tolerance * tolerance;
    auto num_vertices = poly.mesh->vertex_count(face_idx);

    for (unsigned i = 0; i < num_vertices; ++i) {
        auto v_idx = poly.mesh->face_vertex_index(face_idx, i);
        auto v = poly.mesh->vertices[v_idx];

        if (edyn::distance_sqr(v, intersection) < tolerance_sqr) {
            auto normal = edyn::normalize(rayDirLocal);
            dde.drawQuad({normal.x, normal.y, normal.z}, {v.x, v.y, v.z}, 0.015f);
            return;
        }
    }

    for (unsigned i = 0; i < num_vertices; ++i) {
        auto v0_idx = poly.mesh->face_vertex_index(face_idx, i);
        auto v1_idx = poly.mesh->face_vertex_index(face_idx, (i + 1) % num_vertices);
        auto v0 = poly.mesh->vertices[v0_idx];
        auto v1 = poly.mesh->vertices[v1_idx];

        if (edyn::distance_sqr_line(v0, v1 - v0, intersection) < tolerance_sqr) {
            dde.moveTo(v0.x, v0.y, v0.z);
            dde.lineTo(v1.x, v1.y, v1.z);
            return;
        }
    }

    for (size_t i = 0; i < num_vertices; ++i) {
        auto v_idx = poly.mesh->face_vertex_index(face_idx, i);
        auto v = poly.mesh->vertices[v_idx];
        if (i == 0) {
            dde.moveTo(v.x, v.y, v.z);
        } else {
            dde.lineTo(v.x, v.y, v.z);
        }
    }
    dde.close();
}

void drawRaycastResult(DebugDrawEncoder &dde, edyn::compound_shape &compound,
                       const edyn::shape_raycast_result &result,
                       edyn::vector3 rayPos, edyn::vector3 rayDir,
                       edyn::vector3 pos, edyn::quaternion orn) {
    auto rayPosLocal = edyn::to_object_space(rayPos, pos, orn);
    auto rayDirLocal = edyn::rotate(edyn::conjugate(orn), rayDir);
    auto info = std::get<edyn::compound_raycast_info>(result.info_var);
    auto &node = compound.nodes[info.child_index];

    auto bxquat = bx::Quaternion{float(node.orientation.x),
                                 float(node.orientation.y),
                                 float(node.orientation.z),
                                 float(node.orientation.w)};

    float rot[16];
    bx::mtxQuat(rot, bxquat);
    float rotT[16];
    bx::mtxTranspose(rotT, rot);
    float trans[16];
    bx::mtxTranslate(trans, node.position.x, node.position.y, node.position.z);

    float mtx[16];
    bx::mtxMul(mtx, rotT, trans);

    dde.pushTransform(mtx);

    std::visit([&] (auto &&shape) {
        auto child_result = edyn::shape_raycast_result{
            result.fraction,
            result.normal
        };

        std::visit([&] (auto &&child_info) {
            child_result.info_var = child_info;
        }, info.child_info_var);

        drawRaycastResult(dde, shape, child_result, rayPosLocal, rayDirLocal, node.position, node.orientation);
    }, node.shape_var);

    dde.popTransform();
}

void drawRaycastResult(DebugDrawEncoder &dde, edyn::plane_shape &plane,
                       const edyn::shape_raycast_result &result,
                       edyn::vector3 rayPos, edyn::vector3 rayDir,
                       edyn::vector3 pos, edyn::quaternion orn) {

}

void drawRaycastResult(DebugDrawEncoder &dde, edyn::mesh_shape &mesh,
                       const edyn::shape_raycast_result &result,
                       edyn::vector3 rayPos, edyn::vector3 rayDir,
                       edyn::vector3 pos, edyn::quaternion orn) {
    auto tri_idx = std::get<edyn::mesh_raycast_info>(result.info_var).triangle_index;
    auto vertices = mesh.trimesh->get_triangle_vertices(tri_idx);
    auto tri = Triangle {
        bx::Vec3(vertices[0].x, vertices[0].y, vertices[0].z),
        bx::Vec3(vertices[1].x, vertices[1].y, vertices[1].z),
        bx::Vec3(vertices[2].x, vertices[2].y, vertices[2].z)
    };
    dde.draw(tri);
}

void drawRaycastResult(DebugDrawEncoder &dde, edyn::paged_mesh_shape &paged_mesh,
                       const edyn::shape_raycast_result &result,
                       edyn::vector3 rayPos, edyn::vector3 rayDir,
                       edyn::vector3 pos, edyn::quaternion orn) {
    auto info = std::get<edyn::paged_mesh_raycast_info>(result.info_var);
    auto vertices = paged_mesh.trimesh->get_triangle_vertices(info.submesh_index, info.triangle_index);
    auto tri = Triangle {
        bx::Vec3(vertices[0].x, vertices[0].y, vertices[0].z),
        bx::Vec3(vertices[1].x, vertices[1].y, vertices[1].z),
        bx::Vec3(vertices[2].x, vertices[2].y, vertices[2].z)
    };
    dde.draw(tri);
}

void EdynExample::drawRaycast(DebugDrawEncoder &dde) {
    dde.push();
    dde.setColor(0xff0000ff);
    dde.setWireframe(false);

    auto camPos = edyn::vector3{cameraGetPosition().x, cameraGetPosition().y, cameraGetPosition().z};
    auto shapeViews = edyn::get_tuple_of_shape_views(*m_registry);
    auto view = m_registry->view<edyn::shape_raycast_result, edyn::shape_index, edyn::position, edyn::orientation>();
    view.each([&] (entt::entity entity, edyn::shape_raycast_result &result, edyn::shape_index &shapeIndex,
                    edyn::position &pos, edyn::orientation &orn) {
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

        edyn::visit_shape(shapeIndex, entity, shapeViews, [&] (auto &&shape) {
            drawRaycastResult(dde, shape, result, camPos, m_rayDir * m_rayLength, pos, orn);
        });

        dde.popTransform();
    });

    dde.pop();
}

void EdynExample::updatePicking(float viewMtx[16], float proj[16]) {
    float ray_clip[4];
    ray_clip[0] = ( (2.0f * m_mouseState.m_mx) / m_width - 1.0f) * -1.0f;
    ray_clip[1] = ( (1.0f - (2.0f * m_mouseState.m_my) / m_height) ) * -1.0f;
    ray_clip[2] = -1.0f;
    ray_clip[3] =  1.0f;

    float invProjMtx[16];
    bx::mtxInverse(invProjMtx, proj);

    float ray_eye[4];
    bx::vec4MulMtx(ray_eye, ray_clip, invProjMtx);
    ray_eye[2] = -1.0f;
    ray_eye[3] = 0.0f;

    float invViewMtx[16];
    bx::mtxInverse(invViewMtx, viewMtx);

    float ray_world[4];
    bx::vec4MulMtx(ray_world, ray_eye, invViewMtx);

    m_rayDir = edyn::normalize(edyn::vector3{ray_world[0], ray_world[1], ray_world[2]}) * -1.0f;

    if (ImGui::MouseOverArea()) {
        return;
    }

    if (m_mouseState.m_buttons[entry::MouseButton::Left]) {
        const edyn::vector3 cam_pos = {cameraGetPosition().x, cameraGetPosition().y, cameraGetPosition().z};
        const edyn::vector3 cam_at = {cameraGetAt().x, cameraGetAt().y, cameraGetAt().z};

        if (m_pick_entity == entt::null) {
            auto p1 = cam_pos + m_rayDir * m_rayLength;
            auto result = edyn::raycast(*m_registry, cam_pos, p1);

            if (result.entity != entt::null && m_registry->any_of<edyn::dynamic_tag>(result.entity)) {
                auto pick_pos = edyn::lerp(cam_pos, p1, result.fraction);

                auto pos = edyn::get_rigidbody_origin(*m_registry, result.entity);
                auto orn = m_registry->get<edyn::orientation>(result.entity);
                auto pivot = edyn::to_object_space(pick_pos, pos, orn);

                auto pick_def = edyn::rigidbody_def{};
                pick_def.position = pick_pos;
                pick_def.kind = edyn::rigidbody_kind::rb_kinematic;
                pick_def.presentation = false;
                m_pick_entity = edyn::make_rigidbody(*m_registry, pick_def);

                auto &mass = m_registry->get<edyn::mass>(result.entity);
                auto [con_ent, constraint] = edyn::make_constraint<edyn::soft_distance_constraint>(*m_registry, result.entity, m_pick_entity);
                constraint.pivot[0] = pivot;
                constraint.pivot[1] = edyn::vector3_zero;
                constraint.distance = 0;
                constraint.stiffness = std::min(mass.s, edyn::scalar(1e6)) * 100;
                constraint.damping = std::min(mass.s, edyn::scalar(1e6)) * 10;
                m_pick_constraint_entity = con_ent;
            }
        } else {
            // Move kinematic on plane orthogonal to view axis.
            const auto &pick_pos = m_registry->get<edyn::position>(m_pick_entity);
            const auto plane_normal = edyn::normalize(cam_at - cam_pos);
            auto dist = edyn::dot(pick_pos - cam_pos, plane_normal);
            auto s = dist / edyn::dot(m_rayDir, plane_normal);
            auto next_pick_pos = cam_pos + m_rayDir * s;

            if (edyn::distance_sqr(pick_pos, next_pick_pos) > 0.000025) {
                //edyn::update_kinematic_position(*m_registry, m_pick_entity, next_pick_pos, deltaTime);
                m_registry->get<edyn::position>(m_pick_entity) = next_pick_pos;
                m_registry->get_or_emplace<edyn::dirty>(m_pick_entity)
                    .updated<edyn::position>();
            }
        }
    } else if (m_pick_entity != entt::null) {
        m_registry->destroy(m_pick_constraint_entity);
        m_registry->destroy(m_pick_entity);
        m_pick_constraint_entity = entt::null;
        m_pick_entity = entt::null;
    }
}

void EdynExample::updatePhysics(float deltaTime) {
    edyn::update(*m_registry);
}

void EdynExample::togglePausePhysics() {
    setPaused(!m_pause);
}

void EdynExample::stepPhysics() {
    if (m_pause) {
        edyn::step_simulation(*m_registry);
    }
}

void EdynExample::setPaused(bool paused) {
    m_pause = paused;
    edyn::set_paused(*m_registry, m_pause);
}

void EdynExample::updateGUI() {
    imguiBeginFrame(m_mouseState.m_mx
        ,  m_mouseState.m_my
        , (m_mouseState.m_buttons[entry::MouseButton::Left  ] ? IMGUI_MBUT_LEFT   : 0)
        | (m_mouseState.m_buttons[entry::MouseButton::Right ] ? IMGUI_MBUT_RIGHT  : 0)
        | (m_mouseState.m_buttons[entry::MouseButton::Middle] ? IMGUI_MBUT_MIDDLE : 0)
        ,  m_mouseState.m_mz
        , uint16_t(m_width)
        , uint16_t(m_height)
        );

    showExampleDialog(this);
    showSettings();
    showFooter();

    imguiEndFrame();
}

void EdynExample::showSettings() {
    ImGui::SetNextWindowPos(
        ImVec2(m_width - m_width / 4.0f - 10.0f, 10.0f)
        , ImGuiCond_FirstUseEver
        );
    ImGui::SetNextWindowSize(
        ImVec2(m_width / 4.0f, m_height / 3.5f)
        , ImGuiCond_FirstUseEver
        );
    ImGui::Begin("Settings");

    ImGui::SliderInt("Time Step (ms)", &m_fixed_dt_ms, 1, 50);
    ImGui::SliderInt("Velocity Iterations", &m_num_velocity_iterations, 1, 100);
    ImGui::SliderInt("Position Iterations", &m_num_position_iterations, 1, 100);
    ImGui::SliderFloat("Gravity (m/s^2)", &m_gui_gravity, 0, 50, "%.2f");

    ImGui::End();
}

void EdynExample::showFooter() {
    ImGui::SetNextWindowPos(ImVec2(10.0f, m_height - 40.0f));
    ImGui::SetNextWindowSize(ImVec2(m_width - 20.f, 20.f));
    ImGui::SetNextWindowBgAlpha(0.4f);

    ImGui::Begin("Footer", NULL, ImGuiWindowFlags_NoTitleBar |
                                 ImGuiWindowFlags_NoScrollbar |
                                 ImGuiWindowFlags_NoMouseInputs);

    ImGui::Text("%s", m_footer_text.c_str());

    ImGui::End();
}

void EdynExample::updateSettings() {
    auto fixed_dt_ms = static_cast<int>(edyn::get_fixed_dt(*m_registry) * 1000);
    if (fixed_dt_ms != m_fixed_dt_ms) {
        edyn::set_fixed_dt(*m_registry, m_fixed_dt_ms * edyn::scalar(0.001));
    }

    int num_velocity_iterations = edyn::get_solver_velocity_iterations(*m_registry);
    if (num_velocity_iterations != m_num_velocity_iterations) {
        edyn::set_solver_velocity_iterations(*m_registry, m_num_velocity_iterations);
    }

    int num_position_iterations = edyn::get_solver_position_iterations(*m_registry);
    if (num_position_iterations != m_num_position_iterations) {
        edyn::set_solver_position_iterations(*m_registry, m_num_position_iterations);
    }

    if (m_gui_gravity != m_gravity) {
        m_gravity = m_gui_gravity;
        edyn::set_gravity(*m_registry, {0, -m_gravity, 0});
    }
}
