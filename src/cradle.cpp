#include <edyn/edyn.hpp>
#include <entt/entt.hpp>

#include <common/common.h>
#include <common/bgfx_utils.h>
#include <common/imgui/imgui.h>
#include <common/debugdraw/debugdraw.h>
#include <common/camera.h>

namespace
{

void draw(DebugDrawEncoder &dde, const edyn::sphere_shape &sh) {
    Sphere sphere;
    sphere.center = {0,0,0};
    sphere.radius = sh.radius;
    dde.draw(sphere);
}

class ExampleCradle : public entry::AppI
{
public:
	ExampleCradle(const char* _name, const char* _description, const char* _url)
		: entry::AppI(_name, _description, _url)
	{

	}

	void init(int32_t _argc, const char* const* _argv, uint32_t _width, uint32_t _height) override
	{
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

        // Init DebugDraw to draw a grid and spheres.
        ddInit();

		imguiCreate();

        cameraCreate();

		cameraSetPosition({ 0.0f, 2.0f, -12.0f });
		cameraSetVerticalAngle(0.0f);

        m_timeOffset = bx::getHPCounter();

        registry.reset();

        auto& world = registry.ctx_or_set<edyn::world>(registry);

        // Create entities.
        
        auto def = edyn::rigidbody_def();
        def.kind = edyn::rigidbody_kind::rb_dynamic;
        def.presentation = true;
        def.friction = 0.1;
        def.restitution = 0.9;

        auto def_st = edyn::rigidbody_def();
        def_st.kind = edyn::rigidbody_kind::rb_static;

        for (int i = 0; i < 5; ++i) {
            def.position = {i * 0.4, 0, 0};
            def.linvel = edyn::vector3_zero;
            def.angvel = edyn::vector3_zero;
            def.mass = 100;
            def.shape_opt = {edyn::sphere_shape{0.2}};
            def.update_inertia();
            auto ent = edyn::make_rigidbody(registry, def);

            registry.assign<edyn::linacc>(ent, edyn::gravity_earth);

            def_st.position = def.position + edyn::vector3_y * 2;
            auto ent_st = edyn::make_rigidbody(registry, def_st);

            auto constraint = edyn::distance_constraint();
            constraint.pivot[0] = edyn::vector3_zero;
            constraint.pivot[1] = edyn::vector3_zero;
            constraint.distance = 3;
            constraint.stiffness = 1e10;
            constraint.damping = 1e3;
            edyn::make_constraint(registry, constraint, ent, ent_st);

            if (i == 4) {
                auto &pos = registry.get<edyn::position>(ent);
                pos.x += 2;
                pos.y = 2;
            }
        }
	}

	virtual int shutdown() override
	{
        // Cleanup.
		ddShutdown();

		imguiDestroy();

		cameraDestroy();

		// Shutdown bgfx.
		bgfx::shutdown();

		return 0;
	}

	bool update() override
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
        auto& world = registry.ctx<edyn::world>();
        world.update(std::min(deltaTime, 0.1f));

        // Draw stuff.
        DebugDrawEncoder dde;
        dde.begin(0);

        // Grid.
        dde.drawGrid(Axis::Y, { 0.0f, 0.0f, 0.0f });

        // Draw entities.
        auto view = registry.view<const edyn::shape, const edyn::present_position, const edyn::present_orientation>();
        view.each([&] (auto ent, auto &sh, auto &pos, auto &orn) {
            dde.push();

            auto quat = bx::Quaternion{float(orn.x), float(orn.y), float(-orn.z), float(orn.w)};
            float rot[16];
            bx::mtxQuat(rot, quat);
            float trans[16];
            bx::mtxTranslate(trans, pos.x, pos.y, pos.z);

            float mtx[16];
            bx::mtxMul(mtx, rot, trans);

            dde.setTransform(mtx);
            
            std::visit([&] (auto &&s) {
                draw(dde, s);
            }, sh.var);

            dde.pop();
        });

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

                if (pick_entity == entt::null) {                    
                    auto view = registry.view<const edyn::present_position, const edyn::mass>();
                    view.each([&] (auto ent, auto &pos, auto &mass) {
                        if (pick_entity != entt::null) {
                            return;
                        }

                        const auto v = pos - cam_pos;
                        const auto s = edyn::dot(v, ray_dir);

                        if (s > 0) {
                            const auto dist = std::sqrt(edyn::length2(v) - s * s);
                            if (dist < 1) {
                                const auto plane_normal = edyn::normalize(cam_at - cam_pos);
                                auto cam_dist = edyn::dot(pos - cam_pos, plane_normal);
                                auto t = cam_dist / edyn::dot(ray_dir, plane_normal);
                                auto pick_pos = cam_pos + ray_dir * t;

                                pick_entity = registry.create();
                                registry.assign<edyn::kinematic_tag>(pick_entity);
                                registry.assign<edyn::position>(pick_entity, pick_pos);
                                registry.assign<edyn::orientation>(pick_entity, edyn::quaternion_identity);
                                registry.assign<edyn::linvel>(pick_entity, edyn::vector3_zero);
                                registry.assign<edyn::angvel>(pick_entity, edyn::vector3_zero);
                                registry.assign<edyn::mass>(pick_entity, EDYN_SCALAR_MAX);
                                registry.assign<edyn::inertia>(pick_entity, edyn::vector3_max);

                                auto &orientation = registry.get<edyn::orientation>(ent);
                                auto pivot = edyn::rotate(edyn::inverse(orientation), pick_pos - pos);

                                auto constraint = edyn::distance_constraint();
                                constraint.pivot[0] = pivot;
                                constraint.pivot[1] = edyn::vector3_zero;
                                constraint.distance = 0;
                                constraint.stiffness = mass * 100;
                                constraint.damping = mass * 10;
                                pick_constraint_entity = edyn::make_constraint(registry, constraint, ent, pick_entity);
                            }
                        }
                    });
                } else {
                    const auto &pick_pos = registry.get<const edyn::position>(pick_entity);
                    const auto plane_normal = edyn::normalize(cam_at - cam_pos);
                    auto dist = edyn::dot(pick_pos - cam_pos, plane_normal);
                    auto s = dist / edyn::dot(ray_dir, plane_normal);
                    auto next_pick_pos = cam_pos + ray_dir * s;
                    edyn::update_kinematic_position(registry, pick_entity, next_pick_pos, deltaTime);
                }
            } else if (pick_entity != entt::null) {
                registry.destroy(pick_constraint_entity);
                pick_constraint_entity = entt::null;
                registry.destroy(pick_entity);
                pick_entity = entt::null;
            }
        }

        return true;
	}

	entry::MouseState m_mouseState;

	uint32_t m_width;
	uint32_t m_height;
	uint32_t m_debug;
	uint32_t m_reset;
	int64_t m_timeOffset;

    entt::registry registry;
    entt::entity pick_entity {entt::null};
    entt::entity pick_constraint_entity {entt::null};
};

} // namespace

ENTRY_IMPLEMENT_MAIN(
	  ExampleCradle
	, "00-cradle"
	, "Newton's Cradle."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
	);
