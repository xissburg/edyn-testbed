#include "edyn_example.hpp"
#include <edyn/constraints/constraint.hpp>

void ContactStarted(entt::registry &registry, entt::entity entity);
void ContactEnded(entt::registry &registry, entt::entity entity);

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
        // Create entities.
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 1;
        floor_def.friction = 0.5;
        floor_def.shape_opt = {edyn::plane_shape{{0, 1, 0}, 0}};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some polyhedrons.
        auto dyn_def = edyn::rigidbody_def();
        dyn_def.kind = edyn::rigidbody_kind::rb_dynamic;
        dyn_def.continuous_contacts = true;
        dyn_def.mass = 1000;
        auto obj_path = "../../../edyn-testbed/resources/rock.obj";
        //dyn_def.shape_opt = {edyn::polyhedron_shape(obj_path)};
        //dyn_def.shape_opt = {edyn::box_shape{0.2,0.2,0.2}};
        dyn_def.shape_opt = {edyn::cylinder_shape{0.2,0.2}};
        //dyn_def.shape_opt = {edyn::sphere_shape{0.2}};
        dyn_def.update_inertia();
        dyn_def.position = {0.0, 0.2, 0.0};
        dyn_def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), edyn::pi * -0.5);
        edyn::make_rigidbody(*m_registry, dyn_def);

        dyn_def.mass = 100;
        dyn_def.position = {0.1, 0.8, 0.0};
        obj_path = "../../../edyn-testbed/resources/cylinder.obj";
        //dyn_def.shape_opt = {edyn::polyhedron_shape(obj_path)};
        //dyn_def.shape_opt = {edyn::box_shape{0.2,0.2,0.2}};
        //dyn_def.shape_opt = {edyn::cylinder_shape{0.1,0.2}};
        dyn_def.shape_opt = {edyn::capsule_shape{0.1, 0.4}};
        dyn_def.update_inertia();
        dyn_def.orientation = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 1, 0}), edyn::pi * 0.);
        edyn::make_rigidbody(*m_registry, dyn_def);

        m_registry->on_construct<edyn::constraint_impulse>().connect<&ContactStarted>();
        m_registry->on_destroy<edyn::contact_point>().connect<&ContactEnded>();

        m_pause = true;
        auto& world = m_registry->ctx<edyn::world>();
        world.set_paused(m_pause);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExamplePolyhedrons
    , "08-polyhedrons"
    , "Polyhedrons."
    , "https://bkaradzic.github.io/bgfx/examples.html#cubes"
    );


