#include <edyn/edyn.hpp>
#include <entt/entt.hpp>

#include <common/common.h>
#include <common/bgfx_utils.h>
#include <common/imgui/imgui.h>
#include <common/debugdraw/debugdraw.h>
#include <common/camera.h>

namespace
{

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

struct attract : edyn::vector3 {
    edyn::scalar strength {10};
};

void update_attract(entt::registry& registry, edyn::scalar dt) {
    auto view = registry.view<edyn::position, edyn::linvel, attract>();
    view.each([&] (auto, auto &pos, auto &vel, auto &attr) {
        auto d = attr - pos;
        auto l2 = std::max(edyn::length2(d), EDYN_EPSILON);
        auto l = std::sqrt(l2);
        auto dn = d / l;
        vel += dn * (attr.strength * dt / l2);
    });
}

class ExampleParticles : public entry::AppI
{
public:
	ExampleParticles(const char* _name, const char* _description, const char* _url)
		: entry::AppI(_name, _description, _url)
	{
        auto& world = registry.set<edyn::world>(registry);
        world.update_sink().connect<&update_attract>(registry);
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

        // Create a few particle entities.
        {
            const auto ent = registry.create();
            registry.assign<edyn::position>(ent, 3, 3, 0);
            registry.assign<edyn::current_position>(ent);
            registry.assign<edyn::linvel>(ent, 0, 4, -1);
            registry.assign<edyn::mass>(ent, 100);
            registry.assign<edyn::gravity>(ent);
        }

        {
            const auto ent = registry.create();
            registry.assign<edyn::position>(ent, -7, 3.2, 4.2);
            registry.assign<edyn::current_position>(ent);
            registry.assign<edyn::linvel>(ent, 0, 2.1, 0);
            registry.assign<edyn::mass>(ent, 1e10);
            registry.assign<edyn::gravity>(ent);
        }

        {
            const auto ent = registry.create();
            registry.assign<edyn::position>(ent, 0, 3, 0);
            registry.assign<edyn::current_position>(ent);
            registry.assign<edyn::linvel>(ent, 0, 0, 0);
            registry.assign<edyn::mass>(ent, 1e12);
            registry.assign<edyn::gravity>(ent);
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

        float viewMtx[16];
        cameraGetViewMtx(viewMtx);

        float proj[16];

        // Set view and projection matrix for view 0.
        {
            bx::mtxProj(proj, 60.0f, float(m_width)/float(m_height), 0.1f, 10000.0f, bgfx::getCaps()->homogeneousDepth);

            bgfx::setViewTransform(0, viewMtx, proj);
            bgfx::setViewRect(0, 0, 0, uint16_t(m_width), uint16_t(m_height) );
        }

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
        auto view = registry.view<edyn::current_position>();
        view.each([&] (auto ent, auto &pos) {
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
            float mtx[16];
            bx::mtxMul(mtx, scale, trans);

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
};

} // namespace

ENTRY_IMPLEMENT_MAIN(
	  ExampleParticles
	, "01-particles"
	, "Particle entities orbiting one another with gravity."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
	);
