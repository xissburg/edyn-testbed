#include "edyn_example.hpp"

class ExampleHinge : public EdynExample
{
public:
	ExampleHinge(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

    void createScene() override 
    {
        // Create entities.
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 1;
        floor_def.friction = 0.5;
        floor_def.shape_opt = {edyn::plane_shape{{0, 1, 0}, 0}};
        edyn::make_rigidbody(*m_registry, floor_def);
        
        // Add some cylinders.
        auto def = edyn::rigidbody_def();
        def.presentation = true;
        def.restitution = 0;
        def.friction = 0.8;
        def.mass = 100;
        def.shape_opt = {edyn::cylinder_shape{0.2, 0.5}};
        def.update_inertia();

        def.position = {0, 2, 0};
        def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), edyn::pi);
        auto entA = edyn::make_rigidbody(*m_registry, def);

        def.position = {0, 2, 0.8};
        def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), edyn::pi/2);
        auto entB = edyn::make_rigidbody(*m_registry, def);

        auto [hinge_ent, hinge] = edyn::make_constraint<edyn::hinge_constraint>(*m_registry, entA, entB);
        hinge.pivot[0] = {0, 0, 0.4};
        hinge.pivot[1] = {0, 0, -0.4};
        hinge.set_axis(m_registry->get<edyn::orientation>(entA), edyn::vector3_z, -edyn::vector3_z);
	}
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleHinge
	, "08-hinge"
	, "Hinge constraint."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
	);
