#include "edyn_example.hpp"
#include <dear-imgui/imgui.h>
#include <edyn/comp/dirty.hpp>
#include <edyn/math/constants.hpp>
#include <fenv.h>

#include <iostream>

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

    m_timeOffset = bx::getHPCounter();

    m_registry.reset(new entt::registry);

    m_registry->on_construct<edyn::island>().connect<&OnCreateIsland>();
    m_registry->on_destroy<edyn::island>().connect<&OnDestroyIsland>();

    edyn::init();

    // Setup world.
    auto &world = m_registry->set<edyn::world>(*m_registry);
    m_fixed_dt_ms = static_cast<int>(world.get_fixed_dt() * 1000);
    m_gui_gravity = m_gravity = -edyn::gravity_earth.y;

    // Input bindings
    m_bindings = (InputBinding*)BX_ALLOC(entry::getAllocator(), sizeof(InputBinding)*3);
    m_bindings[0].set(entry::Key::KeyP, entry::Modifier::None, 1, cmdTogglePause,  this);
    m_bindings[1].set(entry::Key::KeyL, entry::Modifier::None, 1, cmdStepSimulation, this);
    m_bindings[2].end();

    inputAddBindings("base", m_bindings);

    createScene();
}

int EdynExample::shutdown()
{
    // Cleanup.
    ddShutdown();

    imguiDestroy();

    cameraDestroy();

    inputRemoveBindings("base");
	BX_FREE(entry::getAllocator(), m_bindings);

    // Shutdown bgfx.
    bgfx::shutdown();

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

    int64_t now = bx::getHPCounter() - m_timeOffset;
    static int64_t last = now;
    const int64_t frameTime = now - last;
    last = now;
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

    updateGUI();

    updateSettings();

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

            if (m_registry->has<edyn::sleeping_tag>(ent)) {
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
            bx::mtxTranslate(trans, pos.x, pos.y, pos.z);

            float mtx[16];
            bx::mtxMul(mtx, rotT, trans);

            dde.pushTransform(mtx);

            edyn::visit_shape(sh_idx, ent, shape_views_tuple, [&] (auto &&s) {
                draw(dde, s);
            });

            dde.drawAxis(0, 0, 0, 0.15);

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

    // Draw static entities.
    {
        auto view = m_registry->view<edyn::shape_index, edyn::position, edyn::orientation, edyn::static_tag>();
        view.each([&] (auto ent, auto &sh_idx, auto &pos, auto &orn) {
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

            dde.drawAxis(0, 0, 0, 0.15);
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

    dde.end();

    // Advance to next frame. Rendering thread will be kicked to
    // process submitted rendering primitives.
    bgfx::frame();

    if (!ImGui::MouseOverArea()) {
        if (!!m_mouseState.m_buttons[entry::MouseButton::Left]) {
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

            const edyn::vector3 ray_dir = edyn::normalize(edyn::vector3{ray_world[0], ray_world[1], ray_world[2]}) * -1.0f;
            const edyn::vector3 cam_pos = {cameraGetPosition().x, cameraGetPosition().y, cameraGetPosition().z};
            const edyn::vector3 cam_at = {cameraGetAt().x, cameraGetAt().y, cameraGetAt().z};

            if (m_pick_entity == entt::null) {
                entt::entity picked_entity = entt::null;

                auto view = m_registry->view<edyn::present_position, edyn::mass>();
                view.each([&] (auto ent, auto &pos, auto &mass) {
                    if (picked_entity != entt::null || mass == EDYN_SCALAR_MAX) {
                        return;
                    }

                    const auto v = pos - cam_pos;
                    const auto s = edyn::dot(v, ray_dir);

                    if (s > 0) {
                        const auto dist = std::sqrt(edyn::length_sqr(v) - s * s);
                        if (dist < 1) {
                            picked_entity = ent;
                            // Do not create kinematic entity and constraint while iterating view.
                            // Do it afterwards to avoid possible undefined behavior when changing
                            // the storage while iterating.
                        }
                    }
                });

                if (picked_entity != entt::null) {
                    const auto plane_normal = edyn::normalize(cam_at - cam_pos);
                    auto &pos = view.get<edyn::present_position>(picked_entity);
                    auto cam_dist = edyn::dot(pos - cam_pos, plane_normal);
                    auto t = cam_dist / edyn::dot(ray_dir, plane_normal);
                    auto pick_pos = cam_pos + ray_dir * t;
                    auto &orientation = m_registry->get<edyn::orientation>(picked_entity);
                    auto pivot = edyn::rotate(edyn::conjugate(orientation), pick_pos - pos);

                    auto pick_def = edyn::rigidbody_def{};
                    pick_def.position = pick_pos;
                    pick_def.kind = edyn::rigidbody_kind::rb_kinematic;
                    m_pick_entity = edyn::make_rigidbody(*m_registry, pick_def);

                    auto &mass = view.get<edyn::mass>(picked_entity);
                    auto [con_ent, constraint] = edyn::make_constraint<edyn::soft_distance_constraint>(*m_registry, picked_entity, m_pick_entity);
                    constraint.pivot[0] = pivot;
                    constraint.pivot[1] = edyn::vector3_zero;
                    constraint.distance = 0;
                    constraint.stiffness = std::min(mass.s, edyn::scalar(1e6)) * 100;
                    constraint.damping = std::min(mass.s, edyn::scalar(1e6)) * 10;
                    m_pick_constraint_entity = con_ent;
                }
            } else {
                const auto &pick_pos = m_registry->get<edyn::position>(m_pick_entity);
                const auto plane_normal = edyn::normalize(cam_at - cam_pos);
                auto dist = edyn::dot(pick_pos - cam_pos, plane_normal);
                auto s = dist / edyn::dot(ray_dir, plane_normal);
                auto next_pick_pos = cam_pos + ray_dir * s;

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

    return true;
}

void EdynExample::updatePhysics(float deltaTime) {
    auto& world = m_registry->ctx<edyn::world>();
    world.update();
}

void EdynExample::togglePausePhysics() {
    m_pause = !m_pause;
    auto& world = m_registry->ctx<edyn::world>();
    world.set_paused(m_pause);
}

void EdynExample::stepPhysics() {
    auto& world = m_registry->ctx<edyn::world>();
    world.step();
}

void EdynExample::setPaused(bool paused) {
    m_pause = paused;
    auto& world = m_registry->ctx<edyn::world>();
    world.set_paused(m_pause);
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

    ImGui::Text("Press 'P' to pause and 'L' to step simulation while paused.");

    ImGui::End();
}

void EdynExample::updateSettings() {
    auto &world = m_registry->ctx<edyn::world>();
    auto fixed_dt_ms = static_cast<int>(world.get_fixed_dt() * 1000);
    if (fixed_dt_ms != m_fixed_dt_ms) {
        world.set_fixed_dt(m_fixed_dt_ms * edyn::scalar(0.001));
    }

    if (m_gui_gravity != m_gravity) {
        m_gravity = m_gui_gravity;

        auto view = m_registry->view<edyn::linacc>();
        view.each([&] (entt::entity entity, edyn::linacc &acc) {
            acc.y = -m_gravity;
            m_registry->get_or_emplace<edyn::dirty>(entity).updated<edyn::linacc>();
        });
    }
}
