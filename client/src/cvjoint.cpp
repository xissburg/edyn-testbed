#include "edyn_example.hpp"

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
        auto static_def = edyn::rigidbody_def();
        static_def.kind = edyn::rigidbody_kind::rb_static;
        auto static_ent = edyn::make_rigidbody(*m_registry, static_def);

        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.material->restitution = 0;
        def.material->friction = 0.8;
        def.mass = 3000;
        def.shape = edyn::box_shape{0.5, 0.1, 0.1};
        def.position = {0, 2, 0};
        auto entityA = edyn::make_rigidbody(*m_registry, def);

        edyn::make_constraint<edyn::hinge_constraint>(*m_registry, entityA, static_ent,
            [&](edyn::hinge_constraint &hinge) {
                hinge.pivot[1] = def.position;
                hinge.set_axes({1, 0, 0}, {1, 0, 0});
                hinge.torque = 1;
                hinge.damping = 3;
            });

        def.mass = 100;
        def.position = {0.65, 1.35, 0};
        def.orientation = edyn::quaternion_axis_angle({0, 0, 1}, edyn::to_radians(-90));
        auto entityB = edyn::make_rigidbody(*m_registry, def);

        edyn::exclude_collision(*m_registry, entityA, entityB);

        edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, entityA, entityB,
            [&](edyn::cvjoint_constraint &cvjoint) {
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
            });
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleCvjoint
    , "19-cvjoint"
    , "CV joint constraint."
    , "https://github.com/xissburg/edyn"
    );
