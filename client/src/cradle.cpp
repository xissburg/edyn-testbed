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
        auto def = edyn::rigidbody_def();
        def.material->friction = 0;
        def.material->restitution = 1;
        def.linvel = edyn::vector3_zero;
        def.angvel = edyn::vector3_zero;
        def.mass = 100;
        def.shape = edyn::sphere_shape{0.2};
        def.update_inertia();

        auto def_st = edyn::rigidbody_def();
        def_st.kind = edyn::rigidbody_kind::rb_static;

        const size_t n = 8;
        auto height = edyn::scalar(1);

        for (size_t i = 0; i < n; ++i) {
            def.position = {edyn::scalar(i * 0.4), 0, 0};

            if (i >= n - 1) {
                auto angle = edyn::pi * 0;
                def.position.x += std::cos(angle) * height;
                def.position.y = height - std::sin(angle) * height;
            }

            auto ent = edyn::make_rigidbody(*m_registry, def);

            def_st.position = {edyn::scalar(i * 0.4), height, 0.4};
            auto dist = std::sqrt(edyn::square(height) + edyn::square(def_st.position.z));

            {
                auto ent_st = edyn::make_rigidbody(*m_registry, def_st);
                edyn::make_constraint<edyn::distance_constraint>(*m_registry, ent, ent_st, [&](edyn::distance_constraint &con) {
                    con.pivot[0] = edyn::vector3_zero;
                    con.pivot[1] = edyn::vector3_zero;
                    con.distance = dist;
                });
            }

            {
                def_st.position.z *= -1;
                auto ent_st = edyn::make_rigidbody(*m_registry, def_st);
                edyn::make_constraint<edyn::distance_constraint>(*m_registry, ent, ent_st, [&](edyn::distance_constraint &con) {
                    con.pivot[0] = edyn::vector3_zero;
                    con.pivot[1] = edyn::vector3_zero;
                    con.distance = dist;
                });
            }
        }
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleCradle
    , "09-cradle"
    , "Newton's Cradle."
    , "https://github.com/xissburg/edyn-testbed"
    );
