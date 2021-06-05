#include "edyn_example.hpp"

class ExamplePolyhedrons : public EdynExample
{
public:
    ExamplePolyhedrons(const char* _name, const char* _description, const char* _url)
      : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExamplePolyhedrons() {}

    void createScene() override
    {
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 1;
        floor_def.friction = 0.5;
        floor_def.shape_opt = {edyn::plane_shape{{0, 1, 0}, 0}};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some polyhedrons.
        std::vector<edyn::rigidbody_def> defs;

        auto dyn_def = edyn::rigidbody_def();
        dyn_def.mass = 100;
        dyn_def.restitution = 0;
        dyn_def.friction = 0.7;
        dyn_def.continuous_contacts = true;

        dyn_def.shape_opt = {
            edyn::polyhedron_shape("../../../edyn-testbed/resources/box.obj",
                                   edyn::vector3_zero, // position offset
                                   edyn::quaternion_identity, // orientation
                                   {1.5, 1.8, 2}) // scaling
        };
        dyn_def.update_inertia();
        dyn_def.position = {0.0, 0.5, 0.0};
        dyn_def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), edyn::pi * -0.5);
        defs.push_back(dyn_def);

        dyn_def.shape_opt = {
            edyn::polyhedron_shape("../../../edyn-testbed/resources/cylinder.obj")
        };
        dyn_def.update_inertia();
        dyn_def.position = {-0., 1.2, 0.0};
        dyn_def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{2, 0.6, 1}), edyn::pi * -0.667);
        defs.push_back(dyn_def);

        dyn_def.shape_opt = {
            edyn::compound_shape("../../../edyn-testbed/resources/chain_link.obj")
        };
        dyn_def.update_inertia();
        dyn_def.position = {-0., 1.8, 0.0};
        dyn_def.orientation = edyn::quaternion_identity;
        defs.push_back(dyn_def);

        dyn_def.shape_opt = {
            edyn::polyhedron_shape("../../../edyn-testbed/resources/rock.obj",
                                   edyn::vector3_zero, // position offset
                                   edyn::quaternion_identity, // orientation
                                   {1.1, 0.9, 1.3}) // scaling
        };
        dyn_def.update_inertia();
        dyn_def.position = {0.0, 2.3, 0.0};
        dyn_def.orientation = edyn::quaternion_identity;
        defs.push_back(dyn_def);

        edyn::batch_rigidbodies(*m_registry, defs);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExamplePolyhedrons
    , "07-polyhedrons"
    , "Polyhedrons."
    , "https://github.com/xissburg/edyn-testbed"
    );
