#include "edyn_example.hpp"

class ExampleCradle : public EdynExample
{
public:
	ExampleCradle(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

    void createScene() override
    {
        // This one still doesn't work correctly. Restitution is not being
        // being applied in a manner where the shock would be propagated as
        // expected.
        auto def = edyn::rigidbody_def();
        def.friction = 0;
        def.restitution = 1;
        def.linvel = edyn::vector3_zero;
        def.angvel = edyn::vector3_zero;
        def.mass = 100;
        def.shape = edyn::sphere_shape{0.2};
        def.update_inertia();

        auto def_st = edyn::rigidbody_def();
        def_st.kind = edyn::rigidbody_kind::rb_static;

        const size_t n = 8;

        for (size_t i = 0; i < n; ++i) {
            def.position = {edyn::scalar(i * 0.4), 0, 0};

            if (i == n - 1) {
                auto angle = edyn::pi * 0.25;
                auto radius = edyn::scalar(3);
                def.position.x += std::cos(angle) * radius;
                def.position.y = radius - std::sin(angle) * radius;
            }

            auto ent = edyn::make_rigidbody(*m_registry, def);

            def_st.position = {edyn::scalar(i * 0.4), 3, 0};
            auto ent_st = edyn::make_rigidbody(*m_registry, def_st);

            auto [con_ent, constraint] = edyn::make_constraint<edyn::distance_constraint>(*m_registry, ent, ent_st);
            constraint.pivot[0] = edyn::vector3_zero;
            constraint.pivot[1] = edyn::vector3_zero;
            constraint.distance = 3;
        }
	}
};

ENTRY_IMPLEMENT_MAIN(
	ExampleCradle
	, "09-cradle"
	, "Newton's Cradle."
    , "https://github.com/xissburg/edyn-testbed"
	);
