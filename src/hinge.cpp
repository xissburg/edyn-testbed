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
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.material->restitution = 0;
        def.material->friction = 0.8;
        def.mass = 100;
        def.shape = edyn::box_shape{0.2, 0.5, 0.05};
        def.update_inertia();
        def.continuous_contacts = true;

        std::vector<entt::entity> boxes;

        for (size_t i = 0; i < 8; ++i) {
            auto k = static_cast<edyn::scalar>(i);
            auto z = static_cast<edyn::scalar>(i % 2);
            def.position = {0, 1.f + k * 0.9f, z * 0.1f};
            // Use collision filtering to prevent subsequent entities from colliding.
            def.collision_group = i % 2 == 0 ? 1 : 2;
            def.collision_mask = i % 2 == 0 ? ~2ULL : ~1ULL;
            auto entity = edyn::make_rigidbody(*m_registry, def);
            boxes.push_back(entity);
        }

        // Create hinges.
        for (size_t i = 0; i < size_t(boxes.size() - 1); ++i) {
            auto entityA = boxes[i];
            auto entityB = boxes[i + 1];
            auto z = static_cast<edyn::scalar>((static_cast<int>(i) % 2) * 2 - 1);
            auto [hinge_ent, hinge] = edyn::make_constraint<edyn::hinge_constraint>(*m_registry, entityA, entityB);
            hinge.pivot[0] = {0, 0.45, -0.05f * z};
            hinge.pivot[1] = {0, -0.45, 0.05f * z};
            hinge.axis[0] = {0, 0, 1};
            hinge.axis[1] = {0, 0, 1};
        }
    }
};

ENTRY_IMPLEMENT_MAIN(
	ExampleHinge
	, "05-hinge"
	, "Hinge constraint."
    , "https://github.com/xissburg/edyn"
	);
