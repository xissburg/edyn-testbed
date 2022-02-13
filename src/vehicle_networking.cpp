#include "edyn_example.hpp"
#include "vehicle.hpp"
#include <edyn/comp/orientation.hpp>
#include <edyn/constraints/generic_constraint.hpp>
#include <edyn/constraints/null_constraint.hpp>
#include <edyn/edyn.hpp>
#include <edyn/math/math.hpp>
#include <edyn/math/quaternion.hpp>
#include <edyn/math/vector3.hpp>
#include <edyn/util/constraint_util.hpp>
#include <edyn/util/rigidbody.hpp>

class ExampleVehicle : public EdynExample
{
public:
    ExampleVehicle(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    void createScene() override
    {
        edyn::register_external_components<Vehicle, VehicleSettings, VehicleState, VehicleInput>(*m_registry);
        edyn::set_external_system_pre_step(*m_registry, &UpdateVehicles);

        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        m_vehicle_entity = m_registry->create();
        edyn::tag_external_entity(*m_registry, m_vehicle_entity);

        auto &vehicle = m_registry->emplace<Vehicle>(m_vehicle_entity);

        // Vehicle body.
        auto chassis_def = edyn::rigidbody_def();
        chassis_def.material->restitution = 0.3;
        chassis_def.material->friction = 0.3;
        chassis_def.mass = 600;
        chassis_def.shape = edyn::box_shape{0.65, 0.5, 1.85};
        chassis_def.update_inertia();
        chassis_def.continuous_contacts = true;
        chassis_def.position = {0, 2, 0};
        auto chassis_entity = edyn::make_rigidbody(*m_registry, chassis_def);
        vehicle.chassis_entity = chassis_entity;

        edyn::make_constraint<edyn::null_constraint>(*m_registry, m_vehicle_entity, chassis_entity);

        // Wheels.
        auto wheel_def = edyn::rigidbody_def{};
        wheel_def.material->restitution = 0.6;
        wheel_def.material->friction = 1;
        wheel_def.mass = 50;
        wheel_def.shape = edyn::cylinder_shape{0.4, 0.1};
        wheel_def.update_inertia();

        for (int i = 0; i < 4; ++i) {
            auto lateral = i == 0 || i == 2 ? 1 : -1;
            auto longitudinal = i == 0 || i == 1 ? 1 : -1;
            wheel_def.position = {lateral * 0.9f, 1.5, longitudinal * 1.45f};
            auto wheel_entity = edyn::make_rigidbody(*m_registry, wheel_def);

            edyn::exclude_collision(*m_registry, chassis_entity, wheel_entity);

            auto [con_ent, con] = edyn::make_constraint<edyn::generic_constraint>(*m_registry, chassis_entity, wheel_entity);
            con.pivot[0] = {lateral * 0.8f, 0, longitudinal * 1.45f};
            con.pivot[1] = {-0.1f * lateral, 0, 0};
            con.linear_dofs[1].offset_min = -0.8;
            con.linear_dofs[1].offset_max = 0;
            con.linear_dofs[1].bump_stop_length = 0.1;
            con.linear_dofs[1].bump_stop_stiffness = 250000;
            con.linear_dofs[1].spring_stiffness = 55000;
            con.linear_dofs[1].rest_offset = -0.5;
            con.linear_dofs[1].damping = 2000;
            con.linear_dofs[1].friction_force = 50;
            con.angular_dofs[0].limit_enabled = false;
            con.angular_dofs[0].friction_torque = edyn::to_Nm_per_radian(0.01);

            vehicle.wheel_entity[i] = wheel_entity;
            vehicle.suspension_entity[i] = con_ent;
        }

        m_registry->emplace<VehicleSettings>(m_vehicle_entity);
        m_registry->emplace<VehicleInput>(m_vehicle_entity);
        m_registry->emplace<VehicleState>(m_vehicle_entity);
    }

    void destroyScene() override {
        EdynExample::destroyScene();
        edyn::remove_external_components(*m_registry);
        edyn::remove_external_systems(*m_registry);
    }

    void setSteering(float steering) {
        auto &input = m_registry->get<VehicleInput>(m_vehicle_entity);
        input.steering = steering;
        edyn::refresh<VehicleInput>(*m_registry, m_vehicle_entity);
    }

    void setThrottle(float throttle) {
        auto &input = m_registry->get<VehicleInput>(m_vehicle_entity);
        input.throttle = throttle;
        edyn::refresh<VehicleInput>(*m_registry, m_vehicle_entity);
    }

    void setBrakes(float brakes) {
        auto &input = m_registry->get<VehicleInput>(m_vehicle_entity);
        input.brakes = brakes;
        edyn::refresh<VehicleInput>(*m_registry, m_vehicle_entity);
    }

    void updatePhysics(float deltaTime) override {
        EdynExample::updatePhysics(deltaTime);

        if (inputGetKeyState(entry::Key::Left)) {
            setSteering(-1);
        } else if (inputGetKeyState(entry::Key::Right)) {
            setSteering(1);
        } else {
            setSteering(0);
        }

        if (inputGetKeyState(entry::Key::Up)) {
            setThrottle(1);
        } else {
            setThrottle(0);
        }

        if (inputGetKeyState(entry::Key::Down)) {
            setBrakes(1);
        } else {
            setBrakes(0);
        }
    }

    entt::entity m_vehicle_entity;
};

ENTRY_IMPLEMENT_MAIN(
    ExampleVehicle
    , "00-vehicle-networking"
    , "Networked vehicle."
    , "https://github.com/xissburg/edyn"
    );
