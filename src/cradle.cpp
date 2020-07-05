#include "edyn_example.hpp"

class ExampleCradle : public EDynExample
{
public:
	ExampleCradle(const char* _name, const char* _description, const char* _url)
		: EDynExample(_name, _description, _url)
	{

	}

    void createScene() override 
    {
        //auto& world = m_registry.ctx<edyn::world>();

        // Create entities.
        
        auto def = edyn::rigidbody_def();
        def.kind = edyn::rigidbody_kind::rb_dynamic;
        def.presentation = true;
        def.friction = 0;
        def.restitution = 1;

        auto def_st = edyn::rigidbody_def();
        def_st.kind = edyn::rigidbody_kind::rb_static;

        const size_t n = 8;

        for (size_t i = 0; i < n; ++i) {
            // It only works if the spheres are not touching.
            def.position = {edyn::scalar(i * 0.41), 0, 0};
            def.linvel = edyn::vector3_zero;
            def.angvel = edyn::vector3_zero;
            def.mass = 100;
            def.shape_opt = {edyn::sphere_shape{0.2}};
            def.update_inertia();
            auto ent = edyn::make_rigidbody(m_registry, def);

            def_st.position = def.position + edyn::vector3_y * 3;
            auto ent_st = edyn::make_rigidbody(m_registry, def_st);

            auto constraint = edyn::distance_constraint();
            constraint.pivot[0] = edyn::vector3_zero;
            constraint.pivot[1] = edyn::vector3_zero;
            constraint.distance = 3;
            edyn::make_constraint(m_registry, constraint, ent, ent_st);

            if (i == n - 1) {
                auto &pos = m_registry.get<edyn::position>(ent);
                pos.x += 3;
                pos.y = 3;
            }
        }
	}
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleCradle
	, "03-cradle"
	, "Newton's Cradle."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
	);
