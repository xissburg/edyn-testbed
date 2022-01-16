#include "edyn_example.hpp"
#include <edyn/math/math.hpp>
#include <edyn/shapes/box_shape.hpp>
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

        // Hip
        auto def = edyn::rigidbody_def();
        def.material->restitution = 0.2;
        def.material->friction = 0.5;
        def.mass = 3;
        def.shape = edyn::box_shape{0.2, 0.1, 0.1};
        def.update_inertia();
        def.continuous_contacts = true;
        def.position = {0, 2, 0};
        //def.kind = edyn::rigidbody_kind::rb_static;
        auto hip = edyn::make_rigidbody(*m_registry, def);
        def.kind = edyn::rigidbody_kind::rb_dynamic;

        // Right upper leg
        def.mass = 8;
        def.shape = edyn::box_shape{0.08, 0.25, 0.08};
        def.update_inertia();
        def.position = {0.11, 1.55, 0};
        auto leg_ru = edyn::make_rigidbody(*m_registry, def);

        // Left upper leg
        def.position = {-0.11, 1.55, 0};
        auto leg_lu = edyn::make_rigidbody(*m_registry, def);

        // Right lower leg
        def.mass = 7;
        def.shape = edyn::box_shape{0.07, 0.22, 0.07};
        def.update_inertia();
        def.position = {0.11, 1.08, 0};
        auto leg_rl = edyn::make_rigidbody(*m_registry, def);

        // Left lower leg
        def.position = {-0.11, 1.08, 0};
        auto leg_ll = edyn::make_rigidbody(*m_registry, def);

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
        def.mass = 8;
        def.shape = edyn::box_shape{0.21, 0.12, 0.12};
        def.update_inertia();
        def.position = {0, 2.47, 0};
        auto chest = edyn::make_rigidbody(*m_registry, def);

        // Neck
        def.mass = 1;
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

        // Right shoulder
        def.mass = 1;
        def.shape = edyn::box_shape{0.08, 0.08, 0.08};
        def.update_inertia();
        def.position = {0.14, 2.53, 0};
        auto shoulder_r = edyn::make_rigidbody(*m_registry, def);

        // Left shoulder
        def.position = {-0.14, 2.53, 0};
        auto shoulder_l = edyn::make_rigidbody(*m_registry, def);

        // Right upper arm
        def.mass = 2.5;
        def.shape = edyn::box_shape{0.11, 0.06, 0.06};
        def.update_inertia();
        def.position = {0.33, 2.53, 0};
        auto arm_ru = edyn::make_rigidbody(*m_registry, def);

        // Left upper arm
        def.position = {-0.33, 2.53, 0};
        auto arm_lu = edyn::make_rigidbody(*m_registry, def);

        // Right lower arm
        def.mass = 2;
        def.shape = edyn::box_shape{0.11, 0.05, 0.05};
        def.update_inertia();
        def.position = {0.55, 2.53, 0};
        auto arm_rl = edyn::make_rigidbody(*m_registry, def);

        // Left lower arm
        def.position = {-0.55, 2.53, 0};
        auto arm_ll = edyn::make_rigidbody(*m_registry, def);

        edyn::exclude_collision(*m_registry, hip, leg_ru);
        edyn::exclude_collision(*m_registry, hip, leg_lu);
        edyn::exclude_collision(*m_registry, leg_ru, leg_rl);
        edyn::exclude_collision(*m_registry, leg_lu, leg_ll);
        edyn::exclude_collision(*m_registry, hip, lower_back);
        edyn::exclude_collision(*m_registry, mid_back, lower_back);
        edyn::exclude_collision(*m_registry, mid_back, chest);
        edyn::exclude_collision(*m_registry, neck, chest);
        edyn::exclude_collision(*m_registry, neck, head);
        edyn::exclude_collision(*m_registry, chest, shoulder_r);
        edyn::exclude_collision(*m_registry, mid_back, shoulder_r);
        edyn::exclude_collision(*m_registry, lower_back, shoulder_r);
        edyn::exclude_collision(*m_registry, arm_ru, shoulder_r);
        edyn::exclude_collision(*m_registry, chest, arm_ru);
        edyn::exclude_collision(*m_registry, arm_rl, arm_ru);
        edyn::exclude_collision(*m_registry, chest, shoulder_l);
        edyn::exclude_collision(*m_registry, mid_back, shoulder_l);
        edyn::exclude_collision(*m_registry, lower_back, shoulder_l);
        edyn::exclude_collision(*m_registry, arm_lu, shoulder_l);
        edyn::exclude_collision(*m_registry, chest, arm_lu);
        edyn::exclude_collision(*m_registry, arm_ll, arm_lu);

        // Hip - Right upper leg
        for (auto leg : std::array{leg_ru, leg_lu}) {
            edyn::scalar side = leg == leg_ru ? 1 : -1;

            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, hip, leg);
            cone_con.pivot[0] = {side * edyn::scalar(0.11), 0, 0};
            cone_con.pivot[1] = {0, -0.25, 0};
            cone_con.frame[0] = edyn::matrix3x3_columns(-edyn::vector3_y, edyn::vector3_x, -edyn::vector3_z);
            cone_con.frame[1] = edyn::matrix3x3_columns(-edyn::vector3_y, edyn::vector3_x, -edyn::vector3_z);
            cone_con.span[0] = std::tan(edyn::to_radians(30));
            cone_con.span[1] = std::tan(edyn::to_radians(60));

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, hip, leg);
            cvjoint.pivot[0] = {side * edyn::scalar(0.11), 0, 0};
            cvjoint.pivot[1] = {0, 0.25, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, edyn::vector3_x, -edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, edyn::vector3_x, -edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-60);
            cvjoint.twist_max = edyn::to_radians(30);
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(hip),
                m_registry->get<edyn::orientation>(leg));
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

        // Hip-lower back
        {
            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, hip, lower_back);
            cone_con.pivot[0] = {0, 0.1, 0};
            cone_con.pivot[1] = {0, 0.09, 0};
            cone_con.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.span[0] = std::tan(edyn::to_radians(10));
            cone_con.span[1] = std::tan(edyn::to_radians(20));

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, hip, lower_back);
            cvjoint.pivot[0] = {0, 0.1, 0};
            cvjoint.pivot[1] = {0, -0.09, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-10);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(hip),
                m_registry->get<edyn::orientation>(lower_back));
        }

        // Mid back-lower back
        {
            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, lower_back, mid_back);
            cone_con.pivot[0] = {0, 0.09, 0};
            cone_con.pivot[1] = {0, 0.085, 0};
            cone_con.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.span[0] = std::tan(edyn::to_radians(12));
            cone_con.span[1] = std::tan(edyn::to_radians(26));

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, lower_back, mid_back);
            cvjoint.pivot[0] = {0, 0.09, 0};
            cvjoint.pivot[1] = {0, -0.085, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-12);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(lower_back),
                m_registry->get<edyn::orientation>(mid_back));
        }

        // Mid back-chest
        {
            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, mid_back, chest);
            cone_con.pivot[0] = {0, 0.085, 0};
            cone_con.pivot[1] = {0, 0.12, 0};
            cone_con.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.span[0] = std::tan(edyn::to_radians(10));
            cone_con.span[1] = std::tan(edyn::to_radians(20));

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
        }

        // Neck-chest
        {
            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, chest, neck);
            cone_con.pivot[0] = {0, 0.12, 0};
            cone_con.pivot[1] = {0, 0.05, 0};
            cone_con.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.span[0] = std::tan(edyn::to_radians(12));
            cone_con.span[1] = std::tan(edyn::to_radians(30));

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, chest, neck);
            cvjoint.pivot[0] = {0, 0.12, 0};
            cvjoint.pivot[1] = {0, -0.05, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-23);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(chest),
                m_registry->get<edyn::orientation>(neck));
        }

        // Neck-head
        {
            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, neck, head);
            cone_con.pivot[0] = {0, 0.05, 0};
            cone_con.pivot[1] = {0, 0.13, 0.05};
            cone_con.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.span[0] = std::tan(edyn::to_radians(12));
            cone_con.span[1] = std::tan(edyn::to_radians(30));

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, neck, head);
            cvjoint.pivot[0] = {0, 0.05, 0};
            cvjoint.pivot[1] = {0, -0.13, 0.05};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-23);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(neck),
                m_registry->get<edyn::orientation>(head));
        }

        // Chest-shoulder
        for (auto shoulder : std::array{shoulder_l, shoulder_r}) {
            edyn::scalar side = shoulder == shoulder_l ? -1 : 1;

            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, chest, shoulder);
            cone_con.pivot[0] = {edyn::scalar(0.06) * side, 0.08, 0};
            cone_con.pivot[1] = {edyn::scalar(0.08) * side, 0, 0};
            cone_con.frame[0] = edyn::matrix3x3_columns(side * edyn::vector3_x, edyn::vector3_y, side * edyn::vector3_z);
            cone_con.frame[1] = edyn::matrix3x3_columns(side * edyn::vector3_x, edyn::vector3_y, side * edyn::vector3_z);
            cone_con.span[0] = std::tan(edyn::to_radians(30));
            cone_con.span[1] = std::tan(edyn::to_radians(30));

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, chest, shoulder);
            cvjoint.pivot[0] = {edyn::scalar(0.06) * side, 0.08, 0};
            cvjoint.pivot[1] = {edyn::scalar(-0.08) * side, 0, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(side * edyn::vector3_x, edyn::vector3_y, side * edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(side * edyn::vector3_x, edyn::vector3_y, side * edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-5);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(chest),
                m_registry->get<edyn::orientation>(shoulder));
        }

        // Shoulder-arm
        for (auto pair : std::array{std::make_pair(shoulder_l, arm_lu), std::make_pair(shoulder_r, arm_ru)}) {
            edyn::scalar side = pair.first == shoulder_l ? -1 : 1;

            auto [cone_ent, cone_con] = edyn::make_constraint<edyn::cone_constraint>(*m_registry, pair.first, pair.second);
            cone_con.pivot[0] = {edyn::scalar(0.08) * side, 0, 0};
            cone_con.pivot[1] = {edyn::scalar(0.11) * side, 0, 0};
            cone_con.frame[0] = edyn::matrix3x3_columns(side * edyn::vector3_x, edyn::vector3_y, side * edyn::vector3_z);
            cone_con.frame[1] = edyn::matrix3x3_columns(side * edyn::vector3_x, edyn::vector3_y, side * edyn::vector3_z);
            cone_con.span[0] = std::tan(edyn::to_radians(30));
            cone_con.span[1] = std::tan(edyn::to_radians(30));

            auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, pair.first, pair.second);
            cvjoint.pivot[0] = {edyn::scalar(0.08) * side, 0, 0};
            cvjoint.pivot[1] = {edyn::scalar(-0.11) * side, 0, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(side * edyn::vector3_x, edyn::vector3_y, side * edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(side * edyn::vector3_x, edyn::vector3_y, side * edyn::vector3_z);
            cvjoint.twist_min = edyn::to_radians(-45);
            cvjoint.twist_max = -cvjoint.twist_min;
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(pair.first),
                m_registry->get<edyn::orientation>(pair.second));
        }

        // Elbows
        for (auto arms : std::array{std::make_pair(arm_ru, arm_rl), std::make_pair(arm_lu, arm_ll)}) {
            edyn::scalar side = arms.first == arm_ru ? 1 : -1;

            auto [hinge_ent, hinge] = edyn::make_constraint<edyn::hinge_constraint>(*m_registry, arms.first, arms.second);
            hinge.pivot[0] = {edyn::scalar(0.11) * side, 0, 0};
            hinge.pivot[1] = {edyn::scalar(-0.11) * side, 0, 0};
            hinge.set_axes({0, side * 1, 0}, {0, side * 1, 0});
            hinge.angle_min = 0;
            hinge.angle_max = edyn::to_radians(140);
            hinge.damping = 0.2;
            hinge.friction_torque = 0.1;
            hinge.bump_stop_angle = edyn::to_radians(10);
            hinge.bump_stop_stiffness = 30;
            hinge.reset_angle(
                m_registry->get<edyn::orientation>(arms.first),
                m_registry->get<edyn::orientation>(arms.second));
        }
    }
};

ENTRY_IMPLEMENT_MAIN(
	ExampleRagDoll
	, "22-ragdoll"
	, "Rag doll."
    , "https://github.com/xissburg/edyn"
	);
