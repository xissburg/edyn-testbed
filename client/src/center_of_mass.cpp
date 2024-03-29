#include "edyn_example.hpp"

class ExampleCenterOfMass : public EdynExample
{
public:
    ExampleCenterOfMass(const char* _name, const char* _description, const char* _url)
      : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleCenterOfMass() {}

    void createScene() override
    {
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some dynamic rigid bodies.
        auto dyn_def = edyn::rigidbody_def();
        dyn_def.mass = 100;
        dyn_def.material->restitution = 0;
        dyn_def.material->friction = 0.7;

        {
            dyn_def.shape = edyn::box_shape{0.2, 0.2, 0.2};
            dyn_def.center_of_mass = {0.1, 0.1, 0.1};
            dyn_def.position = {0.0, 0.5, 0.0};
            edyn::make_rigidbody(*m_registry, dyn_def);
        }

        {
            dyn_def.shape = edyn::cylinder_shape{0.15, 0.2, edyn::coordinate_axis::z};
            dyn_def.center_of_mass = {0.01, 0.05, -0.1};
            dyn_def.position = {0.0, 1, 0.0};
            edyn::make_rigidbody(*m_registry, dyn_def);
        }

        {
            dyn_def.shape = edyn::sphere_shape{0.25};
            dyn_def.center_of_mass = {0.0, 0.2, 0.02};
            dyn_def.position = {0.0, 2, 0.0};
            edyn::make_rigidbody(*m_registry, dyn_def);
        }
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleCenterOfMass
    , "14-center-of-mass"
    , "Shifting the center of mass."
    , "https://github.com/xissburg/edyn-testbed"
    );
