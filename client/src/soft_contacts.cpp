#include "edyn_example.hpp"

class ExampleSoftContacts : public EdynExample
{
public:
    ExampleSoftContacts(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleSoftContacts() {}

    void createScene() override
    {
        // Create entities.
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 0;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some soft spheres.
        auto def = edyn::rigidbody_def();
        def.mass = 100;
        def.material->friction = 0.8;
        def.material->spin_friction = 0.0005;
        def.material->roll_friction = 0.0002;
        def.shape = edyn::sphere_shape{0.4};

        const size_t n = 5;

        for (size_t i = 0; i < n; ++i) {
            def.material->stiffness = (1 - edyn::scalar(i) / n) * 100000;
            def.material->damping = 600;
            auto x = (edyn::scalar(i) - edyn::scalar(n)/2) * edyn::scalar(1.6);
            def.position = {x, 3, 0};
            edyn::make_rigidbody(*m_registry, def);
        }
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleSoftContacts
    , "31-soft-contacts"
    , "Soft Contacts."
    , "https://github.com/xissburg/edyn-testbed"
    );
