#include "edyn_example.hpp"

class ExampleCylinders : public EdynExample
{
public:
	ExampleCylinders(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

    virtual ~ExampleCylinders() {}

	void createScene() override
	{
        // Create entities.
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 1;
        floor_def.friction = 0.5;
        floor_def.shape_opt = {edyn::plane_shape{edyn::normalize(edyn::vector3{0.01, 1, 0}), 0}};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some cylinders.
        auto def = edyn::rigidbody_def();
        def.kind = edyn::rigidbody_kind::rb_static;
        def.restitution = 0;
        def.friction = 0.8;
        def.shape_opt = {edyn::cylinder_shape{0.2, 0.2}};
        def.position = {0, 0.5, 0};
        def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), 3 * edyn::pi/2);
        edyn::make_rigidbody(*m_registry, def);

        auto dyn_def = edyn::rigidbody_def();
        dyn_def.kind = edyn::rigidbody_kind::rb_dynamic;
        dyn_def.mass = 100;
        dyn_def.shape_opt = {edyn::cylinder_shape{0.2, 0.2}};
        dyn_def.update_inertia();
        dyn_def.position = {0.1, 0.9, -0.05};
        //dyn_def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{1, 0, 0}), edyn::pi * -30 / 180) * edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), edyn::pi / 2);
        //dyn_def.position = {0.1, 1, 0};
        dyn_def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 1, 0}), -edyn::pi * 0.5);
        edyn::make_rigidbody(*m_registry, dyn_def);

        /* const size_t n = 10;
        for (size_t i = 0; i < n; ++i) {
            def.restitution = edyn::scalar(i) / n;
            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * 0.8, 5, 0};
            edyn::make_rigidbody(*m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * 0.8, 6, 0};
            edyn::make_rigidbody(*m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * 0.8, 7, 0};
            edyn::make_rigidbody(*m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * 0.8, 8, 0};
            edyn::make_rigidbody(*m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * 0.8, 9, 0};
            edyn::make_rigidbody(*m_registry, def);
        } */

        m_registry->on_construct<edyn::contact_point>().connect<&EdynExample::onConstructContactPoint>(*this);

        m_pause = true;
        auto& world = m_registry->ctx<edyn::world>();
        world.set_paused(m_pause);
	}
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleCylinders
	, "04-cylinders"
	, "Cylinders."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
	);


