#include <edyn/edyn.hpp>
#include <entt/entt.hpp>

#include <common/common.h>
#include <common/bgfx_utils.h>
#include <common/imgui/imgui.h>
#include <common/debugdraw/debugdraw.h>
#include <common/camera.h>

#include <edyn/comp/delta_linvel.hpp>
#include <edyn/comp/delta_angvel.hpp>

#include <iostream>

struct PosColorVertex
{
	float m_x;
	float m_y;
	float m_z;
	uint32_t m_abgr;

	static void init()
	{
		ms_layout
			.begin()
			.add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
			.add(bgfx::Attrib::Color0,   4, bgfx::AttribType::Uint8, true)
			.end();
	};

	static bgfx::VertexLayout ms_layout;
};

bgfx::VertexLayout PosColorVertex::ms_layout;

static PosColorVertex s_axesVertices[] =
{
	{-1.0f,  0.0f,  0.0f, 0xff0000ff },
	{ 1.0f,  0.0f,  0.0f, 0xff0000ff },
	{ 0.0f, -1.0f,  0.0f, 0xff00ff00 },
	{ 0.0f,  1.0f,  0.0f, 0xff00ff00 },
	{ 0.0f,  0.0f, -1.0f, 0xffff0000 },
	{ 0.0f,  0.0f,  1.0f, 0xffff0000 },
};

static const uint16_t s_axesLineList[] =
{
	0, 1,
	2, 3,
	4, 5
};

class ExampleParticles : public entry::AppI
{
public:
	ExampleParticles(const char* _name, const char* _description, const char* _url)
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

        // Create vertex stream declaration.
		PosColorVertex::init();

		// Create static vertex buffer.
		m_vbh = bgfx::createVertexBuffer(
			  bgfx::makeRef(s_axesVertices, sizeof(s_axesVertices) )
			, PosColorVertex::ms_layout
			);

        // Create static index buffer for line list rendering.
		m_ibh = bgfx::createIndexBuffer(bgfx::makeRef(s_axesLineList, sizeof(s_axesLineList)));

        // Create program from shaders.
		m_program = loadProgram("vs_cubes", "fs_cubes");

        // Init DebugDraw to draw a grid
        ddInit();

		imguiCreate();

        cameraCreate();

		cameraSetPosition({ 0.0f, 2.0f, -12.0f });
		cameraSetVerticalAngle(0.0f);

        m_timeOffset = bx::getHPCounter();

        registry.reset();

        auto& world = registry.ctx_or_set<edyn::world>(registry);
        
        // Create a few particle entities.
        std::vector<entt::entity> entities;        
        
        auto def = edyn::rigidbody_def();
        def.presentation = true;
        def.gravity = edyn::vector3_zero;

        def.position = {3, 3, 0};
        def.linvel = {0, 4, -1};
        def.angvel = {1, 3, 0.1};
        def.mass = 100;
        def.inertia = {40, 40, 40};
        entities.push_back(edyn::make_rigidbody(registry, def));
    
        def.position = {-7, 3.2, 4.2};
        def.linvel = {0, 2.1, 0};
        def.angvel = edyn::vector3_zero;
        def.mass = 1e10;
        def.inertia = {1e9, 1e9, 1e9};
        entities.push_back(edyn::make_rigidbody(registry, def));
        
        def.position = {0, 3, 0};
        def.linvel = {0, 0, 0};
        def.angvel = edyn::vector3_zero;
        def.mass = 1e12;
        def.inertia = {1e11, 1e11, 1e11};
        entities.push_back(edyn::make_rigidbody(registry, def));

        for (size_t i = 0; i < entities.size(); ++i) {
            for (size_t j = i + 1; j < entities.size(); ++j) {
                auto ent = registry.create();
                registry.assign<edyn::relation>(ent, entities[i], entities[j]);
                registry.assign<edyn::gravity>(ent);
            }
        }
	}

	virtual int shutdown() override
	{
        // Cleanup.
        bgfx::destroy(m_ibh);
		bgfx::destroy(m_vbh);
		bgfx::destroy(m_program);

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

        DebugDrawEncoder dde;
        dde.begin(0);
        dde.drawGrid(Axis::Y, { 0.0f, 0.0f, 0.0f });
        dde.end();

        const uint64_t state = 0
                | BGFX_STATE_WRITE_RGB
				| BGFX_STATE_WRITE_A
				| BGFX_STATE_WRITE_Z
				| BGFX_STATE_DEPTH_TEST_LESS
				| BGFX_STATE_CULL_CW
				| BGFX_STATE_MSAA
                | BGFX_STATE_PT_LINES
				;

        // Update physics.
        auto& world = registry.ctx<edyn::world>();
        world.update(deltaTime);

        // Draw entities.
        auto view = registry.view<const edyn::present_position, const edyn::present_orientation>();
        view.each([&] (auto ent, auto &pos, auto &orn) {
            auto quat = bx::Quaternion{float(orn.x), float(orn.y), float(orn.z), float(orn.w)};
            float rot[16];
            bx::mtxQuat(rot, quat);
            float trans[16];
            bx::mtxTranslate(trans, pos.x, pos.y, pos.z);
            float s = .2f;

            // Scale proportional to mass.
            const auto *mass = registry.try_get<edyn::mass>(ent);
            if (mass) {
                s *= pow(*mass, edyn::scalar(0.06));
            }

            float scale[16];
            bx::mtxScale(scale, s);

            float trans_rot[16];
            bx::mtxMul(trans_rot, rot, trans);

            float mtx[16];
            bx::mtxMul(mtx, scale, trans_rot);

            // Set model matrix for rendering.
            bgfx::setTransform(mtx);

            // Set vertex and index buffer.
            bgfx::setVertexBuffer(0, m_vbh);
            bgfx::setIndexBuffer(m_ibh);

            // Set render states.
            bgfx::setState(state);

            // Submit primitive for rendering to view 0.
            bgfx::submit(0, m_program);
        });

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
                    auto view = registry.view<const edyn::present_position>();
                    view.each([&] (auto ent, auto &pos) {
                        if (pick_entity != entt::null) {
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

                                pick_entity = registry.create();
                                registry.assign<edyn::kinematic_tag>(pick_entity);
                                registry.assign<edyn::position>(pick_entity, pick_pos);
                                registry.assign<edyn::orientation>(pick_entity, edyn::quaternion_identity);
                                registry.assign<edyn::linvel>(pick_entity, edyn::vector3_zero);
                                registry.assign<edyn::angvel>(pick_entity, edyn::vector3_zero);
                                registry.assign<edyn::mass>(pick_entity, EDYN_SCALAR_MAX);
                                registry.assign<edyn::inertia>(pick_entity, edyn::vector3_max);

                                auto &orientation = registry.get<edyn::orientation>(ent);
                                auto pivot = edyn::rotate(edyn::conjugate(orientation), pick_pos - pos);
                                pick_constraint_entity = edyn::make_constraint(registry, edyn::point_constraint{{}, pivot, edyn::vector3_zero}, ent, pick_entity);
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
    bgfx::VertexBufferHandle m_vbh;
	bgfx::IndexBufferHandle m_ibh;
	bgfx::ProgramHandle m_program;
	int64_t m_timeOffset;

    entt::registry registry;
    entt::entity pick_entity {entt::null};
    entt::entity pick_constraint_entity {entt::null};
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleParticles
	, "01-particles"
	, "Particle entities orbiting one another with gravity."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
	);
