#include "edyn_example.hpp"

class ExampleCapsules : public EdynExample
{
public:
    ExampleCapsules(const char* _name, const char* _description, const char* _url)
      : EdynExample(_name, _description, _url)
    {}

    virtual ~ExampleCapsules() {}

    void createScene() override
    {
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material = {0, 0.5}; // {restitution, friction}
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some capsules.
        auto def = edyn::rigidbody_def();
        def.continuous_contacts = true;
        def.material->friction = 0.8;
        def.mass = 100;
        def.shape = edyn::capsule_shape{0.2, 0.35};
        def.update_inertia();

        def.material->restitution = 0;
        def.position = {0, 0.7, 0};
        edyn::make_rigidbody(*m_registry, def);

        edyn::make_rigidbody(*m_registry, def);

        def.position = {-0.2, 1.9, 0};
        edyn::make_rigidbody(*m_registry, def);

        def.material->restitution = 0;
        def.position = {-1.5, 0.7, 0};
        def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), 1.4);
        def.angvel = {0, 4, 0};
        edyn::make_rigidbody(*m_registry, def);

        def.orientation = edyn::quaternion_identity;
        def.angvel = {0, 0, 0};
        def.position = {3.1, 1.5, 0};
        edyn::make_rigidbody(*m_registry, def);

        def.shape = edyn::capsule_shape{0.22, 0.5};
        def.position = {3, 0.5, 0};
        edyn::make_rigidbody(*m_registry, def);

        def.position = {2.87, 2.3, 0};
        def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 1, 0}), 1.57);
        edyn::make_rigidbody(*m_registry, def);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleCapsules
    , "02-capsules"
    , "Capsules."
    , "https://github.com/xissburg/edyn-testbed"
    );
