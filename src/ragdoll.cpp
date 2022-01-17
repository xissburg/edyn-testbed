#include "edyn_example.hpp"
#include <edyn/math/constants.hpp>
#include <edyn/math/math.hpp>
#include <edyn/math/matrix3x3.hpp>
#include <edyn/shapes/box_shape.hpp>
#include <edyn/util/moment_of_inertia.hpp>
#include <edyn/util/rigidbody.hpp>

class ExampleRagDoll : public EdynExample
{
public:
	ExampleRagDoll(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

    void createScene() override
    {
        m_fixed_dt_ms = 4;

        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        auto dyn_def = edyn::rigidbody_def();
        dyn_def.mass = 100;
        dyn_def.material->restitution = 0;
        dyn_def.material->friction = 0.7;
        dyn_def.shape =
            edyn::polyhedron_shape("../../../edyn-testbed/resources/rock.obj",
                                   edyn::vector3_zero, // position offset
                                   edyn::quaternion_identity, // orientation
                                   {1.1, 0.9, 1.3}); // scaling
        dyn_def.update_inertia();
        dyn_def.position = {0.0, 0.2, 0.0};
        dyn_def.orientation = edyn::quaternion_identity;
        edyn::make_rigidbody(*m_registry, dyn_def);

        auto rot_z_pi = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi);

        // Hip
        auto def = edyn::rigidbody_def();
        def.material->restitution = 0.2;
        def.material->friction = 0.5;
        def.mass = 3;
        def.shape = edyn::box_shape{0.2, 0.1, 0.1};
        def.update_inertia();
        def.continuous_contacts = true;
        def.position = {0, 2, 0};
        auto hip = edyn::make_rigidbody(*m_registry, def);

        // Right upper leg
        def.mass = 8;
        def.shape = edyn::box_shape{0.08, 0.25, 0.08};
        def.update_inertia();
        def.position = {0.11, 1.75, 0};
        auto leg_ru = edyn::make_rigidbody(*m_registry, def);

        // Left upper leg
        def.position.x *= -1;
        auto leg_lu = edyn::make_rigidbody(*m_registry, def);

        // Right lower leg
        def.mass = 7;
        def.shape = edyn::box_shape{0.06, 0.22, 0.06};
        def.update_inertia();
        def.position = {0.11, 1.28, 0};
        auto leg_rl = edyn::make_rigidbody(*m_registry, def);

        // Left lower leg
        def.position.x *= -1;
        auto leg_ll = edyn::make_rigidbody(*m_registry, def);

        // Right foot
        def.mass = 1;
        def.shape = edyn::box_shape{0.05, 0.04, 0.13};
        def.update_inertia();
        def.position = {0.11, 1.02, -0.06};
        auto foot_r = edyn::make_rigidbody(*m_registry, def);

        // Left foot
        def.position.x *= -1;
        auto foot_l = edyn::make_rigidbody(*m_registry, def);

        // Lower back
        def.mass = 5;
        def.shape = edyn::box_shape{0.19, 0.09, 0.09};
        def.update_inertia();
        def.position = {0, 2.09, 0};
        auto lower_back = edyn::make_rigidbody(*m_registry, def);

        // Mid back
        def.mass = 6;
        def.shape = edyn::box_shape{0.19, 0.085, 0.085};
        def.update_inertia();
        def.position = {0, 2.265, 0};
        auto mid_back = edyn::make_rigidbody(*m_registry, def);

        // Chest
        def.mass = 7;
        def.shape = edyn::box_shape{0.20, 0.12, 0.1};
        def.update_inertia();
        def.position = {0, 2.47, 0};
        auto chest = edyn::make_rigidbody(*m_registry, def);

        // Neck
        def.mass = 2;
        def.shape = edyn::box_shape{0.05, 0.05, 0.05};
        def.update_inertia();
        def.position = {0, 2.64, 0};
        auto neck = edyn::make_rigidbody(*m_registry, def);

        // Head
        def.mass = 4;
        def.shape = edyn::box_shape{0.085, 0.13, 0.11};
        def.update_inertia();
        def.position = {0, 2.84, -0.05};
        auto head = edyn::make_rigidbody(*m_registry, def);

        // Left shoulder
        def.mass = 1.5;
        def.shape = {};
        def.inertia = edyn::diagonal_matrix(edyn::moment_of_inertia_solid_box(def.mass, {0.08, 0.08, 0.07}));
        def.position = {0.14, 2.53, 0};
        auto shoulder_l = edyn::make_rigidbody(*m_registry, def);

        // Right shoulder
        def.position.x *= -1;
        def.orientation = rot_z_pi;
        auto shoulder_r = edyn::make_rigidbody(*m_registry, def);

        // Left upper arm
        def.mass = 2.5;
        def.shape = edyn::box_shape{0.14, 0.05, 0.05};
        def.update_inertia();
        def.position = {0.36, 2.53, 0};
        def.orientation = edyn::quaternion_identity;
        auto arm_lu = edyn::make_rigidbody(*m_registry, def);

        // Right upper arm
        def.position.x *= -1;
        def.orientation = rot_z_pi;
        auto arm_ru = edyn::make_rigidbody(*m_registry, def);

        // Left lower arm
        def.mass = 1;
        def.shape = edyn::box_shape{0.13, 0.04, 0.04};
        def.update_inertia();
        def.position = {0.63, 2.53, 0};
        def.orientation = edyn::quaternion_identity;
        auto arm_ll = edyn::make_rigidbody(*m_registry, def);

        // Right lower arm
        def.position.x *= -1;
        def.orientation = rot_z_pi;
        auto arm_rl = edyn::make_rigidbody(*m_registry, def);

        // Left forearm
        def.shape = {};
        def.position.x *= -1;
        def.orientation = edyn::quaternion_identity;
        auto forearm_l = edyn::make_rigidbody(*m_registry, def);

        // Right forearm
        def.position.x *= -1;
        def.orientation = rot_z_pi;
        auto forearm_r = edyn::make_rigidbody(*m_registry, def);

        // Left hand
        def.mass = 0.5;
        def.shape = edyn::box_shape{0.065, 0.045, 0.045};
        def.update_inertia();
        def.position = {0.825, 2.525, -0.005};
        def.orientation = edyn::quaternion_identity;
        auto hand_l = edyn::make_rigidbody(*m_registry, def);

        // Right lower arm
        def.position.x *= -1;
        def.orientation = rot_z_pi;
        auto hand_r = edyn::make_rigidbody(*m_registry, def);

        edyn::exclude_collision(*m_registry, hip, leg_ru);
        edyn::exclude_collision(*m_registry, hip, leg_lu);
        edyn::exclude_collision(*m_registry, leg_ru, leg_rl);
        edyn::exclude_collision(*m_registry, leg_lu, leg_ll);
        edyn::exclude_collision(*m_registry, foot_l, leg_ll);
        edyn::exclude_collision(*m_registry, foot_r, leg_rl);
        edyn::exclude_collision(*m_registry, hip, lower_back);
        edyn::exclude_collision(*m_registry, mid_back, lower_back);
        edyn::exclude_collision(*m_registry, mid_back, chest);
        edyn::exclude_collision(*m_registry, neck, chest);
        edyn::exclude_collision(*m_registry, neck, head);
        edyn::exclude_collision(*m_registry, chest, arm_ru);
        edyn::exclude_collision(*m_registry, arm_rl, arm_ru);
        edyn::exclude_collision(*m_registry, chest, arm_lu);
        edyn::exclude_collision(*m_registry, arm_ll, arm_lu);
        edyn::exclude_collision(*m_registry, arm_ll, hand_l);
        edyn::exclude_collision(*m_registry, arm_rl, hand_r);

        // Hip - Upper legs
        for (auto leg : std::array{leg_ru, leg_lu}) {
            edyn::scalar side = leg == leg_ru ? 1 : -1;
            auto cone_rot =
                edyn::quaternion_axis_angle({1, 0, 0}, edyn::to_radians(50)) *
                edyn::quaternion_axis_angle({0, 0, 1}, edyn::to_radians(10 * side));

            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, hip, leg);
            cone_con.pivot[0] = {side * edyn::scalar(0.11), 0, 0};
            cone_con.pivot[1] = {0, -0.25, 0};
            cone_con.frame = edyn::matrix3x3_columns(
                edyn::rotate(cone_rot, -edyn::vector3_y),
                edyn::rotate(cone_rot, edyn::vector3_x),
                edyn::rotate(cone_rot, -edyn::vector3_z));
            cone_con.span_tan[0] = std::tan(edyn::to_radians(45));
            cone_con.span_tan[1] = std::tan(edyn::to_radians(70));
            cone_con.bump_stop_stiffness = 5000;
            cone_con.bump_stop_length = 0.05;

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, hip, leg);
            cvjoint.pivot[0] = {side * edyn::scalar(0.11), 0, 0};
            cvjoint.pivot[1] = {0, 0.25, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, edyn::vector3_x, -edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, edyn::vector3_x, -edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(leg == leg_ru ? -80 : -15);
            cvjoint.twist_max = edyn::to_radians(leg == leg_ru ? 15 : 80);

            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(hip),
                m_registry->get<edyn::orientation>(leg));
            cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(0.02);
            cvjoint.twist_damping = cvjoint.bend_damping = edyn::to_Nm_per_radian(0.2);
            cvjoint.twist_bump_stop_angle = edyn::to_radians(4);
            cvjoint.twist_bump_stop_stiffness = edyn::to_Nm_per_radian(5);
        }

        // Knees
        for (auto legs : std::array{std::make_pair(leg_ru, leg_rl), std::make_pair(leg_lu, leg_ll)}) {
            auto [hinge_ent, hinge] = edyn::make_constraint<edyn::hinge_constraint>(*m_registry, legs.first, legs.second);
            hinge.pivot[0] = {0, -0.25, 0};
            hinge.pivot[1] = {0, 0.22, 0};
            hinge.set_axes({1, 0, 0}, {1, 0, 0});
            hinge.angle_min = edyn::to_radians(-140);
            hinge.angle_max = 0;
            hinge.damping = 2;
            hinge.friction_torque = 1;
            hinge.bump_stop_angle = edyn::to_radians(10);
            hinge.bump_stop_stiffness = 30;
            hinge.reset_angle(
                m_registry->get<edyn::orientation>(legs.first),
                m_registry->get<edyn::orientation>(legs.second));
        }

        // Ankles
        for (auto ankle : std::array{std::make_pair(leg_ll, foot_l), std::make_pair(leg_rl, foot_r)}) {
            auto cone_rot = edyn::quaternion_axis_angle({1, 0, 0}, edyn::to_radians(-20));

            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, ankle.first, ankle.second);
            cone_con.pivot[0] = {0, -0.22, 0};
            cone_con.pivot[1] = {0, -0.06, 0.06};
            cone_con.frame = edyn::matrix3x3_columns(
                edyn::rotate(cone_rot, -edyn::vector3_y),
                edyn::rotate(cone_rot, -edyn::vector3_x),
                edyn::rotate(cone_rot, -edyn::vector3_z));
            cone_con.span_tan[0] = std::tan(edyn::to_radians(40));
            cone_con.span_tan[1] = std::tan(edyn::to_radians(70));
            cone_con.bump_stop_stiffness = 5000;
            cone_con.bump_stop_length = 0.05;

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, ankle.first, ankle.second);
            cvjoint.pivot[0] = {0, -0.22, 0};
            cvjoint.pivot[1] = {0, 0.04, 0.06};
            cvjoint.frame[0] = edyn::matrix3x3_columns(-edyn::vector3_y, -edyn::vector3_x, -edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(-edyn::vector3_y, -edyn::vector3_x, -edyn::vector3_z);
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(ankle.first),
                m_registry->get<edyn::orientation>(ankle.second));
            cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(0.002);
            cvjoint.bend_damping = edyn::to_Nm_per_radian(0.03);
        }

        // Hip-lower back
        {
            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, hip, lower_back);
            cone_con.pivot[0] = {0, 0.1, 0};
            cone_con.pivot[1] = {0, 0.09, 0};
            cone_con.frame = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.span_tan[0] = std::tan(edyn::to_radians(10));
            cone_con.span_tan[1] = std::tan(edyn::to_radians(20));
            cone_con.bump_stop_stiffness = 5000;
            cone_con.bump_stop_length = 0.05;

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, hip, lower_back);
            cvjoint.pivot[0] = {0, 0.1, 0};
            cvjoint.pivot[1] = {0, -0.09, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-12);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(hip),
                m_registry->get<edyn::orientation>(lower_back));
            cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(0.02);
            cvjoint.twist_damping = cvjoint.bend_damping = edyn::to_Nm_per_radian(0.2);
            cvjoint.twist_bump_stop_angle = edyn::to_radians(4);
            cvjoint.twist_bump_stop_stiffness = edyn::to_Nm_per_radian(5);
        }

        // Mid back-lower back
        {
            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, lower_back, mid_back);
            cone_con.pivot[0] = {0, 0.09, 0};
            cone_con.pivot[1] = {0, 0.085, 0};
            cone_con.frame = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.span_tan[0] = std::tan(edyn::to_radians(16));
            cone_con.span_tan[1] = std::tan(edyn::to_radians(30));
            cone_con.bump_stop_stiffness = 5000;
            cone_con.bump_stop_length = 0.05;

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, lower_back, mid_back);
            cvjoint.pivot[0] = {0, 0.09, 0};
            cvjoint.pivot[1] = {0, -0.085, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-18);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(lower_back),
                m_registry->get<edyn::orientation>(mid_back));
            cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(0.02);
            cvjoint.twist_damping = cvjoint.bend_damping = edyn::to_Nm_per_radian(0.2);
            cvjoint.twist_bump_stop_angle = edyn::to_radians(4);
            cvjoint.twist_bump_stop_stiffness = edyn::to_Nm_per_radian(5);
        }

        // Mid back-chest
        {
            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, mid_back, chest);
            cone_con.pivot[0] = {0, 0.085, 0};
            cone_con.pivot[1] = {0, 0.12, 0};
            cone_con.frame = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.span_tan[0] = std::tan(edyn::to_radians(18));
            cone_con.span_tan[1] = std::tan(edyn::to_radians(32));
            cone_con.bump_stop_stiffness = 5000;
            cone_con.bump_stop_length = 0.05;

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, mid_back, chest);
            cvjoint.pivot[0] = {0, 0.085, 0};
            cvjoint.pivot[1] = {0, -0.12, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-10);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(mid_back),
                m_registry->get<edyn::orientation>(chest));
            cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(0.02);
            cvjoint.twist_damping = cvjoint.bend_damping = edyn::to_Nm_per_radian(0.2);
            cvjoint.twist_bump_stop_angle = edyn::to_radians(4);
            cvjoint.twist_bump_stop_stiffness = edyn::to_Nm_per_radian(5);
        }

        // Neck-chest
        {
            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, chest, neck);
            cone_con.pivot[0] = {0, 0.12, 0};
            cone_con.pivot[1] = {0, 0.05, 0};
            cone_con.frame = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.span_tan[0] = std::tan(edyn::to_radians(16));
            cone_con.span_tan[1] = std::tan(edyn::to_radians(32));
            cone_con.bump_stop_stiffness = 3000;
            cone_con.bump_stop_length = 0.05;

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, chest, neck);
            cvjoint.pivot[0] = {0, 0.12, 0};
            cvjoint.pivot[1] = {0, -0.05, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-30);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(chest),
                m_registry->get<edyn::orientation>(neck));
            cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(0.02);
            cvjoint.twist_damping = cvjoint.bend_damping = edyn::to_Nm_per_radian(0.2);
            cvjoint.twist_bump_stop_angle = edyn::to_radians(4);
            cvjoint.twist_bump_stop_stiffness = edyn::to_Nm_per_radian(5);
        }

        // Neck-head
        {
            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, neck, head);
            cone_con.pivot[0] = {0, 0.05, 0};
            cone_con.pivot[1] = {0, 0.13, 0.05};
            cone_con.frame = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.span_tan[0] = std::tan(edyn::to_radians(16));
            cone_con.span_tan[1] = std::tan(edyn::to_radians(32));
            cone_con.bump_stop_stiffness = 5000;
            cone_con.bump_stop_length = 0.05;

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, neck, head);
            cvjoint.pivot[0] = {0, 0.05, 0};
            cvjoint.pivot[1] = {0, -0.13, 0.05};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-30);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(neck),
                m_registry->get<edyn::orientation>(head));
            cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(0.02);
            cvjoint.twist_damping = cvjoint.bend_damping = edyn::to_Nm_per_radian(0.2);
            cvjoint.twist_bump_stop_angle = edyn::to_radians(4);
            cvjoint.twist_bump_stop_stiffness = edyn::to_Nm_per_radian(5);
        }

        // Chest-shoulder
        for (auto shoulder : std::array{shoulder_l, shoulder_r}) {
            edyn::scalar side = shoulder == shoulder_l ? 1 : -1;

            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, chest, shoulder);
            cone_con.pivot[0] = {edyn::scalar(0.06) * side, 0.08, 0};
            cone_con.pivot[1] = {0.08, 0, 0};

            auto cone_rot =
                edyn::quaternion_axis_angle({0, 0, 1}, edyn::to_radians(15 * side)) *
                edyn::quaternion_axis_angle({0, 1, 0}, edyn::to_radians(15 * side));
            cone_con.frame = edyn::to_matrix3x3(cone_rot * m_registry->get<edyn::orientation>(shoulder));

            cone_con.span_tan[0] = std::tan(edyn::to_radians(30));
            cone_con.span_tan[1] = std::tan(edyn::to_radians(40));
            cone_con.bump_stop_stiffness = 3000;
            cone_con.bump_stop_length = 0.03;

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, chest, shoulder);
            cvjoint.pivot[0] = {edyn::scalar(0.06) * side, 0.08, 0};
            cvjoint.pivot[1] = {-0.08, 0, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(side * edyn::vector3_x, side * edyn::vector3_y, edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_identity;
            cvjoint.twist_min = edyn::to_radians(-5);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(chest),
                m_registry->get<edyn::orientation>(shoulder));
            cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(0.02);
            cvjoint.twist_damping = cvjoint.bend_damping = edyn::to_Nm_per_radian(0.2);
            cvjoint.twist_bump_stop_angle = edyn::to_radians(2);
            cvjoint.twist_bump_stop_stiffness = edyn::to_Nm_per_radian(5);
        }

        // Shoulder-arm
        for (auto pair : std::array{std::make_pair(shoulder_l, arm_lu), std::make_pair(shoulder_r, arm_ru)}) {
            edyn::scalar side = pair.first == shoulder_l ? 1 : -1;
            auto cone_rot = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 1, -side}), edyn::to_radians(45));

            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, pair.first, pair.second);
            cone_con.pivot[0] = {0.08, 0, 0};
            cone_con.pivot[1] = {0.14, 0, 0};
            cone_con.frame = edyn::matrix3x3_columns(
                edyn::rotate(cone_rot, edyn::vector3_x),
                edyn::rotate(cone_rot, edyn::vector3_y),
                edyn::rotate(cone_rot, edyn::vector3_z));
            cone_con.span_tan[0] = std::tan(edyn::to_radians(45));
            cone_con.span_tan[1] = std::tan(edyn::to_radians(45));
            cone_con.bump_stop_stiffness = 3000;
            cone_con.bump_stop_length = 0.03;

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, pair.first, pair.second);
            cvjoint.pivot[0] = {0.08, 0, 0};
            cvjoint.pivot[1] = {-0.14, 0, 0};
            cvjoint.frame[0] = edyn::matrix3x3_identity;
            cvjoint.frame[1] = edyn::matrix3x3_identity;
            cvjoint.twist_min = edyn::to_radians(-45);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(pair.first),
                m_registry->get<edyn::orientation>(pair.second));
            cvjoint.twist_friction_torque = cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(0.02);
            cvjoint.twist_damping = cvjoint.bend_damping = edyn::to_Nm_per_radian(0.2);
            cvjoint.twist_bump_stop_angle = edyn::to_radians(4);
            cvjoint.twist_bump_stop_stiffness = edyn::to_Nm_per_radian(5);
        }

        // Elbows
        for (auto arms : std::array{std::make_pair(arm_lu, arm_ll), std::make_pair(arm_ru, arm_rl)}) {
            auto [hinge_ent, hinge] = edyn::make_constraint<edyn::hinge_constraint>(*m_registry, arms.first, arms.second);
            hinge.pivot[0] = {0.14, 0, 0};
            hinge.pivot[1] = {-0.13, 0, 0};
            hinge.set_axes({0, 1, 0}, {0, 1, 0});
            hinge.angle_min = 0;
            hinge.angle_max = edyn::to_radians(140);
            hinge.damping = 0.1;
            hinge.friction_torque = 0.02;
            hinge.bump_stop_angle = edyn::to_radians(10);
            hinge.bump_stop_stiffness = edyn::to_Nm_per_radian(5);
            hinge.reset_angle(
                m_registry->get<edyn::orientation>(arms.first),
                m_registry->get<edyn::orientation>(arms.second));
        }

        // Forearm twist
        for (auto pair : std::array{std::make_pair(arm_ll, forearm_l), std::make_pair(arm_rl, forearm_r)}) {
            auto [hinge_ent, hinge] = edyn::make_constraint<edyn::hinge_constraint>(*m_registry, pair.first, pair.second);
            hinge.pivot[0] = {0, 0, 0};
            hinge.pivot[1] = {0, 0, 0};
            hinge.set_axes({1, 0, 0}, {1, 0, 0});
            hinge.angle_min = -edyn::pi_half;
            hinge.angle_max = edyn::pi_half;
            hinge.damping = 0.1;
            hinge.friction_torque = 0.02;
            hinge.bump_stop_angle = edyn::to_radians(10);
            hinge.bump_stop_stiffness = edyn::to_Nm_per_radian(5);
            hinge.reset_angle(
                m_registry->get<edyn::orientation>(pair.first),
                m_registry->get<edyn::orientation>(pair.second));
        }

        // Forearm-hand
        for (auto pair : std::array{std::make_pair(forearm_l, hand_l), std::make_pair(forearm_r, hand_r)}) {
            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, pair.first, pair.second);
            cone_con.pivot[0] = {0.13, 0, 0};
            cone_con.pivot[1] = {0.065, 0.005, 0.005};
            cone_con.frame = edyn::matrix3x3_identity;
            cone_con.span_tan[0] = std::tan(edyn::to_radians(80));
            cone_con.span_tan[1] = std::tan(edyn::to_radians(30));
            cone_con.bump_stop_stiffness = 2000;
            cone_con.bump_stop_length = 0.03;

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, pair.first, pair.second);
            cvjoint.pivot[0] = {0.13, 0, 0};
            cvjoint.pivot[1] = {-0.065, 0.005, 0.005};
            cvjoint.frame[0] = edyn::matrix3x3_identity;
            cvjoint.frame[1] = edyn::matrix3x3_identity;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(pair.first),
                m_registry->get<edyn::orientation>(pair.second));
            cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(0.002);
            cvjoint.bend_damping = edyn::to_Nm_per_radian(0.1);
        }

        setPaused(true);
    }
};

ENTRY_IMPLEMENT_MAIN(
	ExampleRagDoll
	, "00-ragdoll"
	, "Rag doll."
    , "https://github.com/xissburg/edyn"
	);
