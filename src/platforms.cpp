#include "edyn_example.hpp"
#include <edyn/comp/linvel.hpp>
#include <edyn/comp/orientation.hpp>
#include <edyn/math/quaternion.hpp>
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
                                    edyn::scalar(0.4 * i + 1.6),
                                    edyn::scalar(0.4 * k)};
                    edyn::make_rigidbody(*m_registry, def);
                }
            }
        }

        auto plat_def = edyn::rigidbody_def{};
        plat_def.friction = 0.5;
        plat_def.restitution = 0;
        plat_def.kind = edyn::rigidbody_kind::rb_kinematic;
        plat_def.shape_opt = {edyn::box_shape{1, 0.07, 1.2}};
        plat_def.position = {-0.3, 0.5, 0};
        m_square_platform_entity = edyn::make_rigidbody(*m_registry, plat_def);

        plat_def.shape_opt = {edyn::cylinder_shape{1.5, 0.1}};
        plat_def.position = {0.8, 1.2, 0.8};
        plat_def.orientation = edyn::quaternion_axis_angle({0,0,1}, edyn::to_radians(89.1));
        m_disc_platform_entity = edyn::make_rigidbody(*m_registry, plat_def);
	}

    void updatePhysics(float deltaTime) override {
        if (!m_pause) {
            m_total_time += deltaTime;

            auto &linvel = m_registry->get<edyn::linvel>(m_square_platform_entity);
            linvel.x = std::sin(m_total_time * 7) * 4;
            m_registry->get<edyn::position>(m_square_platform_entity) += linvel * deltaTime;
            m_registry->get_or_emplace<edyn::dirty>(m_square_platform_entity)
                .updated<edyn::position, edyn::linvel>();

            auto &angvel = m_registry->get<edyn::angvel>(m_disc_platform_entity);
            angvel.y = edyn::pi * 0.25;
            auto &orn = m_registry->get<edyn::orientation>(m_disc_platform_entity);
            orn = edyn::integrate(orn, angvel, deltaTime);
            m_registry->get_or_emplace<edyn::dirty>(m_disc_platform_entity)
                .updated<edyn::orientation, edyn::angvel>();
        }

        return EdynExample::updatePhysics(deltaTime);
    }

    entt::entity m_square_platform_entity;
    entt::entity m_disc_platform_entity;
    float m_total_time {0};
};

ENTRY_IMPLEMENT_MAIN(
	  ExamplePlatforms
	,"12-platforms"
	, "Kinematic Platforms."
    , "https://github.com/xissburg/edyn-testbed"
    );
