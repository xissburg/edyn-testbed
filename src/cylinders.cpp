#include "edyn_example.hpp"

class ExampleCylinders : public EDynExample
{
public:
	ExampleCylinders(const char* _name, const char* _description, const char* _url)
		: EDynExample(_name, _description, _url)
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
        floor_def.shape_opt = {edyn::plane_shape{{0, 1, 0}, 0}};
        edyn::make_rigidbody(m_registry, floor_def);

        // Add some cylinders.
        auto def = edyn::rigidbody_def();
        def.presentation = true;
        def.friction = 0.8;
        def.mass = 100;
        def.shape_opt = {edyn::cylinder_shape{0.2, 1}};
        def.update_inertia();

        def.restitution = 0;
        def.position = {0, 0.7, 0};
        def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), 1.4);
        def.angvel = {0, 3.14, 0};
        edyn::make_rigidbody(m_registry, def);

        /* const size_t n = 10;
        for (size_t i = 0; i < n; ++i) {
            def.restitution = edyn::scalar(i) / n;
            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * 0.8, 5, 0};
            edyn::make_rigidbody(m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * 0.8, 6, 0};
            edyn::make_rigidbody(m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * 0.8, 7, 0};
            edyn::make_rigidbody(m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * 0.8, 8, 0};
            edyn::make_rigidbody(m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * 0.8, 9, 0};
            edyn::make_rigidbody(m_registry, def);
        } */
	}
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleCylinders
	, "00-cylinders"
	, "Restitution."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
	);


