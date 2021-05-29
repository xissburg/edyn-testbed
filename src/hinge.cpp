#include "edyn_example.hpp"

class ExampleHinge : public EdynExample
{
public:
	ExampleHinge(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

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

        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.restitution = 0;
        def.friction = 0.8;
        def.mass = 100;
        def.shape_opt = {edyn::box_shape{0.2, 0.5, 0.05}};
        def.update_inertia();
        def.continuous_contacts = true;

        std::vector<entt::entity> boxes;

        for (size_t i = 0; i < 6; ++i) {
            auto k = static_cast<edyn::scalar>(i);
            auto z = static_cast<edyn::scalar>(i % 2);
            def.position = {0, 1.f + k * 0.9f, z * 0.15f};
            auto entity = edyn::make_rigidbody(*m_registry, def);
            boxes.push_back(entity);
        }

        for (int i = 0; i < boxes.size() - 1; ++i) {
            auto entityA = boxes[i];
            auto entityB = boxes[i + 1];
            auto z = static_cast<edyn::scalar>((i % 2) * 2 - 1);
            auto [hinge_ent, hinge] = edyn::make_constraint<edyn::hinge_constraint>(*m_registry, entityA, entityB);
            hinge.pivot[0] = {0, 0.45, -0.075f * z};
            hinge.pivot[1] = {0, -0.45, 0.075f * z};
            hinge.set_axis(m_registry->get<edyn::orientation>(entityA), edyn::vector3_z, -edyn::vector3_z);
        }
	  }
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleHinge
	, "05-hinge"
	, "Hinge constraint."
    , "https://github.com/xissburg/edyn"
	);
