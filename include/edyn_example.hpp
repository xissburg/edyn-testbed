#ifndef EDYN_TESTBED_EDYN_EXAMPLE_HPP
#define EDYN_TESTBED_EDYN_EXAMPLE_HPP

#include <edyn/edyn.hpp>
#include <entt/entt.hpp>

#include <common/common.h>
#include <common/bgfx_utils.h>
#include <common/imgui/imgui.h>
#include <common/debugdraw/debugdraw.h>
#include <common/camera.h>
#include <common/entry/input.h>

#include "debugdraw.hpp"

struct ColorComponent {
    uint32_t value;
    operator uint32_t & () {
        return value;
    } 
    operator uint32_t () const {
        return value;
    } 
};

class EDynExample : public entry::AppI
{
public:
	EDynExample(const char* _name, const char* _description, const char* _url)
		: entry::AppI(_name, _description, _url)
	{

	}

    virtual ~EDynExample() {}

	void init(int32_t _argc, const char* const* _argv, uint32_t _width, uint32_t _height) override;

    int shutdown() override;

	bool update() override;

    virtual void createScene() = 0;
    virtual void updatePhysics(float deltaTime);

	entry::MouseState m_mouseState;

	uint32_t m_width;
	uint32_t m_height;
	uint32_t m_debug;
	uint32_t m_reset;
	int64_t m_timeOffset;

    InputBinding* m_bindings;

    bool m_pause;

    entt::registry m_registry;
    entt::entity m_pick_entity {entt::null};
    entt::entity m_pick_constraint_entity {entt::null};
};



#endif // EDYN_TESTBED_EDYN_EXAMPLE_HPP