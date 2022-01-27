#include "edyn_example.hpp"
#include <edyn/constraints/hinge_constraint.hpp>
#include <edyn/edyn.hpp>
#include <edyn/math/math.hpp>
#include <edyn/math/matrix3x3.hpp>
#include <edyn/math/quaternion.hpp>
#include <edyn/util/constraint_util.hpp>
#include <edyn/util/rigidbody.hpp>

class ExampleCvjoint : public EdynExample
{
public:
    ExampleCvjoint(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    void createScene() override
    {
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        auto floor_ent = edyn::make_rigidbody(*m_registry, floor_def);

        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.material->restitution = 0;
        def.material->friction = 0.8;
        def.mass = 3000;
        def.shape = edyn::box_shape{0.5, 0.1, 0.1};
        def.update_inertia();
        def.continuous_contacts = true;
        def.position = {0, 2, 0};
        auto entityA = edyn::make_rigidbody(*m_registry, def);

        auto [hinge_ent, hinge] = edyn::make_constraint<edyn::hinge_constraint>(*m_registry, entityA, floor_ent);
        hinge.pivot[1] = def.position;
        hinge.set_axes({1, 0, 0}, {1, 0, 0});
        hinge.friction_torque = 1;
        hinge.damping = 3;

        def.mass = 100;
        def.update_inertia();
        def.position = {0.65, 1.35, 0};
        def.orientation = edyn::quaternion_axis_angle({0, 0, 1}, edyn::to_radians(-90));
        auto entityB = edyn::make_rigidbody(*m_registry, def);

        edyn::exclude_collision(*m_registry, entityA, entityB);

        auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, entityA, entityB);
        cvjoint.pivot[0] = {0.65, 0, 0};
        cvjoint.pivot[1] = {-0.65, 0, 0};
        cvjoint.twist_min = edyn::to_radians(-180);
        cvjoint.twist_max = edyn::to_radians(180);
        cvjoint.twist_restitution = 0.25;
        cvjoint.twist_bump_stop_angle = edyn::to_radians(30);
        cvjoint.twist_bump_stop_stiffness = edyn::to_Nm_per_radian(0.8);
        cvjoint.twist_friction_torque = edyn::to_Nm_per_radian(0.001);
        cvjoint.twist_damping = edyn::to_Nm_per_radian(0.02);
        cvjoint.twist_rest_angle = edyn::to_radians(15);
        cvjoint.twist_stiffness = edyn::to_Nm_per_radian(0.01);
        cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(0.1);
        cvjoint.bend_damping = edyn::to_Nm_per_radian(1);
        cvjoint.bend_stiffness = edyn::to_Nm_per_radian(10);
        cvjoint.rest_direction = edyn::normalize(edyn::vector3{1, -1, 0});
        cvjoint.reset_angle(
            m_registry->get<edyn::orientation>(entityA),
            m_registry->get<edyn::orientation>(entityB));
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleCvjoint
    , "19-cvjoint"
    , "CV joint constraint."
    , "https://github.com/xissburg/edyn"
    );
