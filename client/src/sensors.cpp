#include "edyn_example.hpp"

static void SensorContactStarted(entt::registry &registry, entt::entity entity) {
    auto &manifold = registry.get<edyn::contact_manifold>(entity);
    for (auto body : manifold.body) {
        if (auto *color = registry.try_get<ColorComponent>(body)) {
            color->value = 0x80ffffff;
        }
    }
}

static void SensorContactEnded(entt::registry &registry, entt::entity entity) {
    auto &manifold = registry.get<edyn::contact_manifold>(entity);
    for (auto body : manifold.body) {
        if (auto *color = registry.try_get<ColorComponent>(body)) {
            color->value = 0x80000000;
        }
    }
}

class ExampleSensors : public EdynExample
{
public:
    ExampleSensors(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    void createScene() override
    {
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 0;
        floor_def.material->friction = 0.8;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        auto dyn_def = edyn::rigidbody_def();
        dyn_def.material->friction = 0.8;
        dyn_def.material->restitution = 1;
        dyn_def.linvel = edyn::vector3_zero;
        dyn_def.angvel = edyn::vector3_zero;
        dyn_def.mass = 100;
        dyn_def.shape = edyn::box_shape{0.2, 0.2, 0.2};
        dyn_def.position = {1, 0, 0};
        edyn::make_rigidbody(*m_registry, dyn_def);

        auto sensor_def = edyn::rigidbody_def();
        sensor_def.kind = edyn::rigidbody_kind::rb_static;
        sensor_def.material.reset(); // Set material to empty to make it a sensor.
        sensor_def.shape = edyn::box_shape{0.8, 0.15, 0.5};
        sensor_def.position = {0.2, 0.15, 0};
        sensor_def.orientation = edyn::quaternion_axis_angle({0, 1, 0}, edyn::to_radians(38));
        auto sensor_ent = edyn::make_rigidbody(*m_registry, sensor_def);
        m_registry->emplace<ColorComponent>(sensor_ent, 0x80000000);

        sensor_def.shape = edyn::cylinder_shape{0.34, 0.5, edyn::coordinate_axis::y};
        sensor_def.position = {1.2, 0.5, 0.9};
        sensor_ent = edyn::make_rigidbody(*m_registry, sensor_def);
        m_registry->emplace<ColorComponent>(sensor_ent, 0x80000000);

        edyn::on_contact_started(*m_registry).connect<&SensorContactStarted>(*m_registry);
        edyn::on_contact_ended(*m_registry).connect<&SensorContactEnded>(*m_registry);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleSensors
    , "23-sensors"
    , "Sensors."
    , "https://github.com/xissburg/edyn-testbed"
    );
