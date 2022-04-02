#include "edyn_example.hpp"
#include <edyn/math/math.hpp>
#include <edyn/math/matrix3x3.hpp>
#include <edyn/math/quaternion.hpp>
#include <edyn/util/rigidbody.hpp>

class ExampleGenericConstraint : public EdynExample
{
public:
    ExampleGenericConstraint(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    void createScene() override
    {
        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.material->restitution = 0.9;
        def.material->friction = 0.8;
        def.shape = edyn::box_shape{0.5, 0.1, 0.1};
        def.continuous_contacts = true;
        def.position = {0, 3, 0};
        def.kind = edyn::rigidbody_kind::rb_static;
        auto entityA = edyn::make_rigidbody(*m_registry, def);

        def.kind = edyn::rigidbody_kind::rb_dynamic;
        def.mass = 100;
        def.update_inertia();
        def.position = {1, 3, 0};
        //def.orientation = edyn::quaternion_axis_angle({1,0,0}, edyn::to_radians(180));
        auto entityB = edyn::make_rigidbody(*m_registry, def);

        edyn::exclude_collision(*m_registry, entityA, entityB);

        auto [con_ent, con] = edyn::make_constraint<edyn::generic_constraint>(*m_registry, entityA, entityB);
        con.pivot[0] = {0.5, 0, 0};
        con.pivot[1] = {-0.5, 0, 0};

        con.linear_dofs[1].offset_min = -1;
        con.linear_dofs[1].offset_max = 0;
        con.linear_dofs[1].bump_stop_length = 0.01;
        con.linear_dofs[1].bump_stop_stiffness = 200000;
        con.linear_dofs[1].spring_stiffness = 18000;
        con.linear_dofs[1].rest_offset = -0.6;
        con.linear_dofs[1].damping = 100;
        con.linear_dofs[1].friction_force = 5;

        con.angular_dofs[0].angle_min = edyn::to_radians(-30);
        con.angular_dofs[0].angle_max = edyn::to_radians(90);

        con.angular_dofs[1].angle_min = edyn::to_radians(-30);
        con.angular_dofs[1].angle_max = edyn::to_radians(30);

        con.angular_dofs[2].angle_min = edyn::to_radians(-15);
        con.angular_dofs[2].angle_max = edyn::to_radians(15);

        con.angular_dofs[0].limit_restitution = 1;
        con.angular_dofs[0].friction_torque = edyn::to_Nm_per_radian(0.01);
        con.angular_dofs[0].damping = edyn::to_Nm_per_radian(0.1);

        con.angular_dofs[1].bump_stop_stiffness = edyn::to_Nm_per_radian(80);
        con.angular_dofs[1].bump_stop_angle = edyn::to_radians(5);
        con.angular_dofs[1].spring_stiffness = edyn::to_Nm_per_radian(0.1);
        con.angular_dofs[1].rest_angle = edyn::to_radians(0);
        con.angular_dofs[1].friction_torque = edyn::to_Nm_per_radian(0.01);
        con.angular_dofs[1].damping = edyn::to_Nm_per_radian(0.1);

        con.angular_dofs[2].limit_restitution = 0.5;
        con.angular_dofs[2].friction_torque = edyn::to_Nm_per_radian(0.001);
        con.angular_dofs[2].damping = edyn::to_Nm_per_radian(0.01);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleGenericConstraint
    , "22-generic"
    , "Generic constraint."
    , "https://github.com/xissburg/edyn"
    );
