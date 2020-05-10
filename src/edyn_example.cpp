#include "edyn_example.hpp"
#include <fenv.h> 

void cmdTogglePause(const void* _userData) {
    ((EDynExample *)_userData)->m_pause = !((EDynExample *)_userData)->m_pause;
}

void cmdStepSimulation(const void* _userData) {
    auto &world = ((EDynExample *)_userData)->m_registry.ctx<edyn::world>();
    world.update(world.fixed_dt);
}

void OnCreateIsland(entt::entity entity, entt::registry &registry, edyn::island &) {
    registry.assign<ColorComponent>(entity, 0xff000000 | (0x00ffffff & rand()));
}

void OnDestroyIsland(entt::entity entity, entt::registry &registry) {
    registry.remove<ColorComponent>(entity);
}

void EDynExample::init(int32_t _argc, const char* const* _argv, uint32_t _width, uint32_t _height)
{
    feenableexcept(FE_INVALID | FE_OVERFLOW | FE_DIVBYZERO);

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

    m_registry.reset();

    m_registry.on_construct<edyn::island>().connect<&OnCreateIsland>();
    m_registry.on_destroy<edyn::island>().connect<&OnDestroyIsland>();

    auto& world = m_registry.ctx_or_set<edyn::world>(m_registry);

    // Input bindings
    m_bindings = (InputBinding*)BX_ALLOC(entry::getAllocator(), sizeof(InputBinding)*3);
    m_bindings[0].set(entry::Key::KeyP, entry::Modifier::None, 1, cmdTogglePause,  this);
    m_bindings[1].set(entry::Key::KeyL, entry::Modifier::None, 1, cmdStepSimulation, this);
    m_bindings[2].end();

    inputAddBindings(getName(), m_bindings);

    createScene();
}

int EDynExample::shutdown()
{
    // Cleanup.
    ddShutdown();

    imguiDestroy();

    cameraDestroy();

    inputRemoveBindings(getName());
	BX_FREE(entry::getAllocator(), m_bindings);

    // Shutdown bgfx.
    bgfx::shutdown();

    return 0;
}

bool EDynExample::update()
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

    ImGui::SetNextWindowPos(
            ImVec2(m_width - m_width / 5.0f - 10.0f, 10.0f)
        , ImGuiCond_FirstUseEver
        );
    ImGui::SetNextWindowSize(
            ImVec2(m_width / 5.0f, m_height / 3.5f)
        , ImGuiCond_FirstUseEver
        );
    ImGui::Begin("Settings"
        , NULL
        );

    ImGui::End();

    imguiEndFrame();

    // Update physics.
    if (!m_pause) {
        auto& world = m_registry.ctx<edyn::world>();
        world.update(std::min(deltaTime, 0.1f));
    }

    bgfx::dbgTextPrintf(0, 1, 0x2f, "Press 'P' to pause and 'L' to step simulation while paused.");

    // Draw stuff.
    DebugDrawEncoder dde;
    dde.begin(0);

    // Grid.
    dde.drawGrid(Axis::Y, { 0.0f, 0.0f, 0.0f });

    // Draw entities.
    {
        auto view = m_registry.view<const edyn::shape, const edyn::present_position, const edyn::present_orientation>();
        view.each([&] (auto ent, auto &sh, auto &pos, auto &orn) {
            dde.push();

            uint32_t color = 0xffffffff;
            
            if (m_registry.has<edyn::sleeping_tag>(ent)) {
                color = 0x80000000;
            } else {
                auto node = m_registry.try_get<edyn::island_node>(ent);
                if (node) {
                    assert(m_registry.valid(node->island_entity));
                    assert(m_registry.has<edyn::island>(node->island_entity));
                    color = m_registry.get<ColorComponent>(node->island_entity);
                }
            }
            dde.setColor(color);

            auto quat = bx::Quaternion{float(orn.x), float(orn.y), float(orn.z), float(orn.w)};
            float rot[16];
            bx::mtxQuat(rot, quat);
            float rotT[16];
            bx::mtxTranspose(rotT, rot);
            float trans[16];
            bx::mtxTranslate(trans, pos.x, pos.y, pos.z);

            float mtx[16];
            bx::mtxMul(mtx, rotT, trans);

            dde.pushTransform(mtx);
            
            std::visit([&] (auto &&s) {
                draw(dde, s);
            }, sh.var);

            dde.popTransform();

            // Draw AABBs.
            std::visit([&] (auto &&s) {
                dde.push();

                uint32_t color = 0xff0000f2;
                dde.setColor(color);
                dde.setWireframe(true);

                auto aabb = s.aabb(pos, orn);
                dde.draw(Aabb{{aabb.min.x, aabb.min.y, aabb.min.z}, {aabb.max.x, aabb.max.y, aabb.max.z}});

                dde.pop();
            }, sh.var);

            dde.pop();
        });
    }

    // Draw constraints.
    {
        auto view = m_registry.view<const edyn::constraint, const edyn::relation>();
        view.each([&] (auto ent, auto &con, auto &rel) {
            std::visit([&] (auto &&c) {
                draw(dde, ent, c, rel, m_registry);
            }, con.var);
        });
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
                auto view = m_registry.view<const edyn::present_position, const edyn::mass>();
                view.each([&] (auto ent, auto &pos, auto &mass) {
                    if (m_pick_entity != entt::null) {
                        return;
                    }

                    const auto v = pos - cam_pos;
                    const auto s = edyn::dot(v, ray_dir);

                    if (s > 0) {
                        const auto dist = std::sqrt(edyn::length_sqr(v) - s * s);
                        if (dist < 1) {
                            const auto plane_normal = edyn::normalize(cam_at - cam_pos);
                            auto cam_dist = edyn::dot(pos - cam_pos, plane_normal);
                            auto t = cam_dist / edyn::dot(ray_dir, plane_normal);
                            auto pick_pos = cam_pos + ray_dir * t;

                            m_pick_entity = m_registry.create();
                            m_registry.assign<edyn::kinematic_tag>(m_pick_entity);
                            m_registry.assign<edyn::position>(m_pick_entity, pick_pos);
                            m_registry.assign<edyn::orientation>(m_pick_entity, edyn::quaternion_identity);
                            m_registry.assign<edyn::linvel>(m_pick_entity, edyn::vector3_zero);
                            m_registry.assign<edyn::angvel>(m_pick_entity, edyn::vector3_zero);
                            m_registry.assign<edyn::mass>(m_pick_entity, EDYN_SCALAR_MAX);
                            m_registry.assign<edyn::inertia>(m_pick_entity, edyn::vector3_max);

                            auto &orientation = m_registry.get<edyn::orientation>(ent);
                            auto pivot = edyn::rotate(edyn::conjugate(orientation), pick_pos - pos);

                            auto constraint = edyn::soft_distance_constraint();
                            constraint.pivot[0] = pivot;
                            constraint.pivot[1] = edyn::vector3_zero;
                            constraint.distance = 0;
                            constraint.stiffness = mass * 100;
                            constraint.damping = mass * 10;
                            m_pick_constraint_entity = edyn::make_constraint(m_registry, constraint, ent, m_pick_entity);
                        }
                    }
                });
            } else {
                const auto &pick_pos = m_registry.get<const edyn::position>(m_pick_entity);
                const auto plane_normal = edyn::normalize(cam_at - cam_pos);
                auto dist = edyn::dot(pick_pos - cam_pos, plane_normal);
                auto s = dist / edyn::dot(ray_dir, plane_normal);
                auto next_pick_pos = cam_pos + ray_dir * s;
                edyn::update_kinematic_position(m_registry, m_pick_entity, next_pick_pos, deltaTime);
            }
        } else if (m_pick_entity != entt::null) {
            m_registry.destroy(m_pick_constraint_entity);
            m_pick_constraint_entity = entt::null;
            m_registry.destroy(m_pick_entity);
            m_pick_entity = entt::null;
        }
    }

    return true;
}