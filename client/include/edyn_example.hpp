#ifndef EDYN_TESTBED_EDYN_EXAMPLE_HPP
#define EDYN_TESTBED_EDYN_EXAMPLE_HPP

#include <edyn/collision/raycast.hpp>
#include <edyn/math/vector3.hpp>
#include <memory>
#include <edyn/edyn.hpp>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>

#include <bgfx/bgfx.h>
#include <common/common.h>
#include <common/bgfx_utils.h>
#include <common/imgui/imgui.h>
#include <common/debugdraw/debugdraw.h>
#include <common/camera.h>
#include <common/entry/input.h>

#include "debugdraw.hpp"

#ifdef EDYN_SOUND_ENABLED
#include <soloud.h>
#endif

struct NullCallback : public bgfx::CallbackI {
    virtual ~NullCallback()	{}

    virtual void fatal(const char* _filePath, uint16_t _line, bgfx::Fatal::Enum _code, const char* _str) override {}
    virtual void traceVargs(const char* _filePath, uint16_t _line, const char* _format, va_list _argList) override {}
    virtual void profilerBegin(const char* /*_name*/, uint32_t /*_abgr*/, const char* /*_filePath*/, uint16_t /*_line*/) override {}
    virtual void profilerBeginLiteral(const char* /*_name*/, uint32_t /*_abgr*/, const char* /*_filePath*/, uint16_t /*_line*/) override {}
    virtual void profilerEnd() override {}
    virtual uint32_t cacheReadSize(uint64_t _id) override { return 0; }
    virtual bool cacheRead(uint64_t _id, void* _data, uint32_t _size) override { return false; }
    virtual void cacheWrite(uint64_t _id, const void* _data, uint32_t _size) override {}
    virtual void screenShot(const char* _filePath, uint32_t _width, uint32_t _height, uint32_t _pitch, const void* _data, uint32_t /*_size*/, bool _yflip) override {}
    virtual void captureBegin(uint32_t _width, uint32_t _height, uint32_t /*_pitch*/, bgfx::TextureFormat::Enum /*_format*/, bool _yflip) override {}
    virtual void captureEnd() override {}
    virtual void captureFrame(const void* _data, uint32_t /*_size*/) override {}
};

struct ColorComponent {
    uint32_t value;
    operator uint32_t & () {
        return value;
    }
    operator uint32_t () const {
        return value;
    }
};

class EdynExample : public entry::AppI
{
public:
    EdynExample(const char* _name, const char* _description, const char* _url)
        : entry::AppI(_name, _description, _url)
    {}

    virtual ~EdynExample() {}

    void init(int32_t _argc, const char* const* _argv, uint32_t _width, uint32_t _height) override;

    int shutdown() override;

    bool update() override;

    virtual void initEdyn();
    virtual void createScene() = 0;
    virtual void destroyScene() {};
    virtual void updatePhysics(float deltaTime);
    void togglePausePhysics();
    void stepPhysics();
    void setPaused(bool);
    void updateGUI();
    void updatePicking(float viewMtx[16], float proj[16]);
    void processRaycast(const edyn::raycast_result &, edyn::vector3 p0, edyn::vector3 p1);
    void onRaycastResult(edyn::raycast_id_type, const edyn::raycast_result &,
                         edyn::vector3, edyn::vector3);
    void showSettings();
    void showFooter();
    void updateSettings();

    void drawRaycast(DebugDrawEncoder &dde);

    entry::MouseState m_mouseState;

    uint32_t m_width;
    uint32_t m_height;
    uint32_t m_debug;
    uint32_t m_reset;
    int64_t m_timestamp;
    NullCallback m_callback;

    InputBinding* m_bindings;

    int m_fixed_dt_ms;
    int m_num_velocity_iterations;
    int m_num_position_iterations;
    float m_gui_gravity;
    edyn::scalar m_gravity;
    bool m_pause;

    std::unique_ptr<entt::registry> m_registry;
    entt::entity m_pick_entity {entt::null};
    entt::entity m_pick_constraint_entity {entt::null};
    bool m_picking {false};
    edyn::vector3 m_rayDir;
    edyn::scalar m_rayLength {100.f};
    edyn::scalar m_rigid_body_axes_size {0.15f};
    bool m_proportional_pick_stiffness {true};

	std::string m_footer_text;
	std::string m_default_footer_text {"Press 'P' to pause and 'L' to step simulation while paused."};

#ifdef EDYN_SOUND_ENABLED
    SoLoud::Soloud m_soloud;
#endif
};



#endif // EDYN_TESTBED_EDYN_EXAMPLE_HPP
