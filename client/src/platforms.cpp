#include "edyn_example.hpp"
#include <edyn/util/rigidbody.hpp>

struct SquarePlatformTag { edyn::scalar angle {0}; };
struct DiscPlatformTag {};

void UpdatePlatforms(entt::registry &registry)
{
    auto dt = edyn::get_fixed_dt(registry);

    // It's important to assign a proper velocity to the kinematic
    // entities for correct constraint behavior, i.e. friction.
    auto square_platform_entity = registry.view<SquarePlatformTag>().front();
    auto angle = (registry.get<SquarePlatformTag>(square_platform_entity).angle += dt * 4);
    registry.get<edyn::linvel>(square_platform_entity).x = std::sin(angle) * -0.8;
    registry.get<edyn::position>(square_platform_entity).x = -0.9 + std::cos(angle) * 0.8;
    registry.patch<edyn::position>(square_platform_entity);

    auto disc_platform_entity = registry.view<DiscPlatformTag>().front();
    auto &disc_w = registry.get<edyn::angvel>(disc_platform_entity);
    registry.patch<edyn::orientation>(disc_platform_entity, [&](auto &&orn) {
        orn = edyn::integrate(orn, disc_w, dt);
    });
}

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
        edyn::register_external_components<SquarePlatformTag, DiscPlatformTag>(*m_registry);
        //edyn::set_pre_step_callback(*m_registry, &UpdatePlatforms);

        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.mass = 10;
        def.material->friction = 0.4;
        def.material->restitution = 0;
        def.shape = edyn::box_shape{0.2, 0.2, 0.2};

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
        plat_def.material->restitution = 0;
        plat_def.material->friction = 0.9;
        plat_def.kind = edyn::rigidbody_kind::rb_kinematic;
        plat_def.shape = edyn::box_shape{1, 0.07, 1.2};
        plat_def.position = {-0.3, 0.5, 0};
        m_square_platform_entity = edyn::make_rigidbody(*m_registry, plat_def);
        //m_registry->emplace<SquarePlatformTag>(square_platform_entity);

        plat_def.shape = edyn::cylinder_shape{1.5, 0.1};
        plat_def.position = {0.8, 1.2, 0.8};
        plat_def.orientation = edyn::quaternion_axis_angle({0,0,1}, edyn::to_radians(89.1));
        plat_def.angvel = {0, edyn::pi * 0.25, 0};
        plat_def.material->friction = 0.5;
        m_disc_platform_entity = edyn::make_rigidbody(*m_registry, plat_def);
    }

    void updatePhysics(float deltaTime) override {
        if (!m_pause) {
            m_total_time += deltaTime;
            auto angle = m_total_time * 4;

            auto pos = m_registry->get<edyn::position>(m_square_platform_entity);
            pos.x = -0.9 + std::cos(angle) * 0.8;
            edyn::set_kinematic_position(*m_registry, m_square_platform_entity, pos, deltaTime);

            auto angvel = edyn::vector3{0, edyn::pi * 0.25, 0};
            auto orn = m_registry->get<edyn::orientation>(m_disc_platform_entity);
            orn = edyn::integrate(orn, angvel, deltaTime);
            edyn::set_kinematic_orientation(*m_registry, m_disc_platform_entity, orn, deltaTime);
        }

        EdynExample::updatePhysics(deltaTime);
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
