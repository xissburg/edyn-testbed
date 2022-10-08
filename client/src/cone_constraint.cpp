#include "edyn_example.hpp"
#include "drawing_properties.hpp"

class ExampleConeConstraint : public EdynExample
{
public:
    ExampleConeConstraint(const char* _name, const char* _description, const char* _url)
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
        def.mass = 100;
        def.shape = edyn::box_shape{0.06, 0.5, 0.06};
        def.position = {0, 2, 0};
        auto entityA = edyn::make_rigidbody(*m_registry, def);

        edyn::make_constraint<edyn::hinge_constraint>(*m_registry, entityA, static_ent, [&](edyn::hinge_constraint &hinge) {
            hinge.pivot[1] = def.position;
            hinge.set_axes({0, 1, 0}, {0, 1, 0});
            hinge.friction_torque = 10;
            hinge.damping = 3;
        });

        def.position = {0.01, 3.3, 0.001};
        auto entityB = edyn::make_rigidbody(*m_registry, def);

        auto cone_ent = edyn::make_constraint<edyn::cone_constraint>(*m_registry, entityA, entityB, [&](edyn::cone_constraint &cone_con) {
            cone_con.pivot[0] = {0, 0.65, 0};
            cone_con.pivot[1] = {0, 0.5, 0};
            cone_con.frame = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cone_con.span_tan[0] = std::tan(edyn::to_radians(30));
            cone_con.span_tan[1] = std::tan(edyn::to_radians(60));
            cone_con.bump_stop_length = 0.3;
            cone_con.bump_stop_stiffness = 5000;
        });

        edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, entityA, entityB, [&](edyn::cvjoint_constraint &cvjoint) {
            cvjoint.pivot[0] = {0, 0.65, 0};
            cvjoint.pivot[1] = {0, -0.65, 0};
            cvjoint.frame[0] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.frame[1] = edyn::matrix3x3_columns(edyn::vector3_y, -edyn::vector3_x, edyn::vector3_z);
            cvjoint.bend_damping = edyn::to_Nm_per_radian(20);
            cvjoint.bend_friction_torque = edyn::to_Nm_per_radian(3);
            cvjoint.reset_angle(
                m_registry->get<edyn::orientation>(entityA),
                m_registry->get<edyn::orientation>(entityB));
        });

        m_registry->emplace<DrawingProperties>(cone_ent);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleConeConstraint
    , "20-cone-constraint"
    , "Cone constraint."
    , "https://github.com/xissburg/edyn"
    );
