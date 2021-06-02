#include "edyn_example.hpp"
#include <edyn/comp/linvel.hpp>
#include <edyn/util/rigidbody.hpp>

class ExamplePlatforms : public EdynExample
{
public:
	ExamplePlatforms(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

    virtual ~ExamplePlatforms() {}

	void createScene() override
	{
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 0;
        floor_def.friction = 0.5;
        floor_def.shape_opt = {edyn::plane_shape{{0, 1, 0}, 0}};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.friction = 0.8;
        def.mass = 10;
        def.restitution = 0;
        def.shape_opt = {edyn::box_shape{0.2, 0.2, 0.2}};
        def.update_inertia();
        def.continuous_contacts = true;

        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                for (int k = 0; k < 5; ++k) {
                    def.position = {edyn::scalar(0.4 * j),
                                    edyn::scalar(0.4 * i + 1),
                                    edyn::scalar(0.4 * k)};
                    edyn::make_rigidbody(*m_registry, def);
                }
            }
        }

        auto plat_def = edyn::rigidbody_def{};
        plat_def.friction = 0.5;
        plat_def.restitution = 0;
        plat_def.kind = edyn::rigidbody_kind::rb_kinematic;
        plat_def.shape_opt = {edyn::box_shape{1, 0.1, 1.2}};
        plat_def.position = {0, 0.5, 0};
        m_platform_entity = edyn::make_rigidbody(*m_registry, plat_def);

        setPaused(true);
	}

    void updatePhysics(float deltaTime) override {
        if (!m_pause) {
            m_total_time += deltaTime;
            auto &vel = m_registry->get<edyn::linvel>(m_platform_entity);
            vel.x = std::sin(m_total_time * 7) * 4;
            auto &pos = m_registry->get<edyn::position>(m_platform_entity);
            pos += vel * deltaTime;
            m_registry->get_or_emplace<edyn::dirty>(m_platform_entity)
                .updated<edyn::position, edyn::linvel>();
        }

        return EdynExample::updatePhysics(deltaTime);
    }

    entt::entity m_platform_entity;
    float m_total_time {0};
};

ENTRY_IMPLEMENT_MAIN(
	  ExamplePlatforms
	,"12-platforms"
	, "Kinematic Platforms."
    , "https://github.com/xissburg/edyn-testbed"
    );
