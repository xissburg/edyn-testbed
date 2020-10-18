#include "edyn_example.hpp"

class ExampleBoxes : public EdynExample
{
public:
	ExampleBoxes(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

    virtual ~ExampleBoxes() {}

	void createScene() override
	{
        // Create entities.
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 0;
        floor_def.friction = 0.5;
        floor_def.shape_opt = {edyn::box_shape{{1, 0.5, 1}}};// {edyn::plane_shape{{0, 1, 0}, 0}};
        floor_def.presentation = true;
        edyn::make_rigidbody(m_registry, floor_def);

        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.presentation = true;
        def.friction = 0.8;
        def.mass = 10;
        def.restitution = 0;
        def.position = {0, 4, 0};
        def.orientation = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi * 0.0);
        def.shape_opt = {edyn::box_shape{0.2, 0.2, 0.2}};
        //def.gravity = {0,0,0};
        def.update_inertia();

        const auto n = 1;

        for (int i = 0; i < n; ++i) {
            def.position = {0, edyn::scalar(1 + i * 1.1), 0};
            edyn::make_rigidbody(m_registry, def);
        }

        //m_pause = true;
	}
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleBoxes
	, "00-boxes"
	, "Boxes."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
	);


