#include "edyn_example.hpp"
#include <edyn/constraints/hinge_constraint.hpp>
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
        def.mass = 100;
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

        def.position = {1.3, 2, 0};
        def.orientation = edyn::quaternion_axis_angle({1, 0, 0}, edyn::to_radians(45));
        auto entityB = edyn::make_rigidbody(*m_registry, def);

        auto [cvjoint_ent, cvjoint] = edyn::make_constraint<edyn::cvjoint_constraint>(*m_registry, entityA, entityB);
        cvjoint.pivot[0] = {0.65, 0, 0};
        cvjoint.pivot[1] = {-0.65, 0, 0};
        cvjoint.frame[1] = cvjoint.frame[1] * edyn::to_matrix3x3(def.orientation);
        cvjoint.angle_min = edyn::to_radians(-270);
        cvjoint.angle_max = edyn::to_radians(270);
        cvjoint.limit_restitution = 0.25;
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
