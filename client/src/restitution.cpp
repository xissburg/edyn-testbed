#include "edyn_example.hpp"

class ExampleRestitution : public EdynExample
{
public:
    ExampleRestitution(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleRestitution() {}

    void createScene() override
    {
        // Create entities.
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some bouncy spheres.
        auto def = edyn::rigidbody_def();
        def.mass = 100;
        def.material->friction = 0.8;
        def.material->spin_friction = 0.0005;
        def.material->roll_friction = 0.0002;
        def.shape = edyn::sphere_shape{0.2};
        //def.linvel = {0, -2, 0};
        //def.angvel = {0, 0, -32};

        const size_t n = 10;

        for (size_t i = 0; i < n; ++i) {
            def.material->restitution = 1 - edyn::scalar(i) / n;
            auto x = (edyn::scalar(i) - edyn::scalar(n)/2) * edyn::scalar(0.8);

            for (size_t j = 0; j < 2; ++j) {
                auto y = edyn::scalar(3 + 0.4 * j);
                def.position = {x, y, 0};
                edyn::make_rigidbody(*m_registry, def);
            }
        }
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleRestitution
    , "08-restitution"
    , "Restitution."
    , "https://github.com/xissburg/edyn-testbed"
    );
