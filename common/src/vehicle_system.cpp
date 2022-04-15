#include "vehicle_system.hpp"
#include "pick_input.hpp"
#include <edyn/comp/action_list.hpp>
#include <edyn/networking/networking_external.hpp>
#include <entt/entity/registry.hpp>
#include <entt/meta/factory.hpp>
#include <entt/core/hashed_string.hpp>
#include <edyn/edyn.hpp>

void RegisterVehicleComponents(entt::registry &registry) {
    using namespace entt::literals;
    entt::meta<Vehicle>().type()
        .data<&Vehicle::chassis_entity, entt::as_ref_t>("chassis_entity"_hs)
        .data<&Vehicle::suspension_entity, entt::as_ref_t>("suspension_entity"_hs)
        .data<&Vehicle::wheel_entity, entt::as_ref_t>("wheel_entity"_hs);
    auto actions = std::tuple<VehicleAction>{};
    edyn::register_external_components<PickInput, Vehicle, VehicleSettings, VehicleState>(registry, actions);
}

void RegisterNetworkedVehicleComponents(entt::registry &registry) {
    auto actions = std::tuple<VehicleAction>{};
    edyn::register_networked_components<
        PickInput,
        Vehicle,
        VehicleSettings,
        VehicleState
    >(registry, actions);
}

entt::entity CreateVehicle(entt::registry &registry) {
    auto vehicle_entity = registry.create();
    edyn::tag_external_entity(registry, vehicle_entity);

    auto &vehicle = registry.emplace<Vehicle>(vehicle_entity);

    // Vehicle body.
    auto chassis_def = edyn::rigidbody_def();
    chassis_def.material->restitution = 0.3;
    chassis_def.material->friction = 0.3;
    chassis_def.mass = 600;
    chassis_def.shape = edyn::box_shape{0.65, 0.5, 1.85};
    chassis_def.update_inertia();
    chassis_def.continuous_contacts = true;
    chassis_def.position = {0, 2, 0};
    auto chassis_entity = edyn::make_rigidbody(registry, chassis_def);
    vehicle.chassis_entity = chassis_entity;

    // Create a connection between vehicle entity and chassis entity to ensure
    // the vehicle entity will be present wherever the chassis goes.
    vehicle.null_con_entity = registry.create();
    edyn::make_constraint<edyn::null_constraint>(vehicle.null_con_entity, registry, vehicle_entity, chassis_entity);

    // Wheels.
    auto wheel_def = edyn::rigidbody_def{};
    wheel_def.material->restitution = 0.6;
    wheel_def.material->friction = 1;
    wheel_def.material->roll_friction = 0.004;
    wheel_def.material->stiffness = 86000;
    wheel_def.material->damping = 800;
    wheel_def.mass = 50;
    wheel_def.shape = edyn::cylinder_shape{0.4, 0.1};
    wheel_def.update_inertia();

    for (int i = 0; i < 4; ++i) {
        auto lateral = i == 0 || i == 2 ? 1 : -1;
        auto longitudinal = i == 0 || i == 1 ? 1 : -1;
        wheel_def.position = {lateral * 0.9f, 1.5, longitudinal * 1.45f};
        auto wheel_entity = edyn::make_rigidbody(registry, wheel_def);

        edyn::exclude_collision(registry, chassis_entity, wheel_entity);

        auto [con_ent, con] = edyn::make_constraint<edyn::generic_constraint>(registry, chassis_entity, wheel_entity);
        con.pivot[0] = {lateral * 0.8f, 0, longitudinal * 1.45f};
        con.pivot[1] = {-0.1f * lateral, 0, 0};
        con.linear_dofs[1].offset_min = -0.8;
        con.linear_dofs[1].offset_max = 0;
        con.linear_dofs[1].bump_stop_length = 0.1;
        con.linear_dofs[1].bump_stop_stiffness = 180000;
        con.linear_dofs[1].spring_stiffness = 40000;
        con.linear_dofs[1].rest_offset = -0.5;
        con.linear_dofs[1].damping = 1100;
        con.linear_dofs[1].friction_force = 10;
        con.angular_dofs[0].limit_enabled = false;
        con.angular_dofs[0].friction_torque = edyn::to_Nm_per_radian(0.006);

        vehicle.wheel_entity[i] = wheel_entity;
        vehicle.suspension_entity[i] = con_ent;
    }

    registry.emplace<VehicleSettings>(vehicle_entity);
    registry.emplace<VehicleState>(vehicle_entity);
    registry.emplace<edyn::action_list<VehicleAction>>(vehicle_entity);
    registry.emplace<edyn::action_history>(vehicle_entity);
    registry.emplace<edyn::continuous>(vehicle_entity).insert(edyn::get_component_index<VehicleState>(registry));

    return vehicle_entity;
}

void ExecuteAction(entt::registry &registry, entt::entity entity, const VehicleSteeringAction &action) {
    auto [settings, state] = registry.get<const VehicleSettings, VehicleState>(entity);
    state.target_steering = action.value * settings.max_steering_angle;
}

void ExecuteAction(entt::registry &registry, entt::entity entity, const VehicleThrottleAction &action) {
    auto [vehicle, state] = registry.get<const Vehicle, VehicleState>(entity);

    for (int i = 0; i < 4; ++i) {
        auto wheel_entity = vehicle.wheel_entity[i];
        auto &wheel_linvel = registry.get<edyn::linvel>(wheel_entity);
        auto &wheel_angvel = registry.get<edyn::angvel>(wheel_entity);
        auto &wheel_orn = registry.get<edyn::orientation>(wheel_entity);
        auto spin_axis = edyn::quaternion_x(wheel_orn);
        auto spin_speed = edyn::dot(wheel_angvel, spin_axis);
        auto longitudinal_speed = edyn::length(edyn::project_direction(wheel_linvel, spin_axis));

        if (std::abs(spin_speed) * 0.25f > longitudinal_speed) {
            state.throttle[i] = 0;
        } else {
            state.throttle[i] = action.value;
        }
    }
}

void ExecuteAction(entt::registry &registry, entt::entity entity, const VehicleBrakeAction &action) {
    auto [vehicle, state] = registry.get<const Vehicle, VehicleState>(entity);

    for (int i = 0; i < 4; ++i) {
        auto wheel_entity = vehicle.wheel_entity[i];
        auto &wheel_linvel = registry.get<edyn::linvel>(wheel_entity);
        auto &wheel_angvel = registry.get<edyn::angvel>(wheel_entity);
        auto &wheel_orn = registry.get<edyn::orientation>(wheel_entity);
        auto spin_axis = edyn::quaternion_x(wheel_orn);
        auto spin_speed = edyn::dot(wheel_angvel, spin_axis);
        auto longitudinal_speed = edyn::length(edyn::project_direction(wheel_linvel, spin_axis));

        if (longitudinal_speed > 2 &&
            std::abs(spin_speed) < edyn::to_radians(1))
        {
            state.brakes[i] = 0;
        } else {
            state.brakes[i] = action.value;
        }
    }
}

void ProcessActions(entt::registry &registry) {
    for (auto [entity, list] : registry.view<edyn::action_list<VehicleAction>>().each()) {
        // Consume actions.
        for (auto &action : list.actions) {
            std::visit([&registry, entity = entity](auto &&containedAction) {
                ExecuteAction(registry, entity, containedAction);
            }, action.var);
        }
    }
}

void ApplySteering(entt::registry &registry, const Vehicle &vehicle,
                   const VehicleSettings &settings, VehicleState &state, edyn::scalar dt) {
    auto steering_direction = state.target_steering > state.steering ? 1 : -1;
    auto steering_increment = std::min(settings.max_steering_rate * dt, std::abs(state.target_steering - state.steering)) * steering_direction;
    state.steering += steering_increment;

    for (int i = 0; i < 2; ++i) {
        auto steering = state.steering;

        // Slight Ackerman effect.
        if ((i == 0 && steering > 0) || (i == 1 && steering < 0)) {
            steering *= 1.1;
        }

        auto &con = registry.get<edyn::generic_constraint>(vehicle.suspension_entity[i]);
        con.frame[0] = edyn::to_matrix3x3(edyn::quaternion_axis_angle({0, 1, 0}, steering));
    }
}

void ApplyThrottle(entt::registry &registry, const Vehicle &vehicle,
                   const VehicleSettings &settings, VehicleState &state, edyn::scalar dt) {
    for (int i = 0; i < 4; ++i) {
        auto wheel_entity = vehicle.wheel_entity[i];
        auto &wheel_orn = registry.get<edyn::orientation>(wheel_entity);
        auto spin_axis = edyn::quaternion_x(wheel_orn);

        auto driving_torque = state.throttle[i] * settings.driving_torque * spin_axis;
        edyn::rigidbody_apply_torque_impulse(registry, vehicle.wheel_entity[i], driving_torque * dt);
    }
}

void ApplyBrakes(entt::registry &registry, const Vehicle &vehicle,
                 const VehicleSettings &settings, VehicleState &state, edyn::scalar dt) {
    for (int i = 0; i < 4; ++i) {
        auto &con = registry.get<edyn::generic_constraint>(vehicle.suspension_entity[i]);
        con.angular_dofs[0].friction_torque = state.brakes[i] * settings.brake_torque + settings.bearing_torque;
    }
}

void UpdateVehicles(entt::registry &registry) {
    ProcessActions(registry);

    auto dt = edyn::get_fixed_dt(registry);
    auto vehicle_view = registry.view<const Vehicle, const VehicleSettings, VehicleState>();

    for (auto [entity, vehicle, settings, state] : vehicle_view.each()) {
        ApplySteering(registry, vehicle, settings, state, dt);
        ApplyThrottle(registry, vehicle, settings, state, dt);
        ApplyBrakes(registry, vehicle, settings, state, dt);
    }
}

std::vector<entt::entity> GetVehicleEntities(entt::registry &registry,
                                             entt::entity vehicle_entity) {
    std::vector<entt::entity> entities;
    entities.push_back(vehicle_entity);

    auto &vehicle = registry.get<Vehicle>(vehicle_entity);
    entities.push_back(vehicle.chassis_entity);
    entities.insert(entities.end(), vehicle.wheel_entity.begin(), vehicle.wheel_entity.end());
    entities.insert(entities.end(), vehicle.suspension_entity.begin(), vehicle.suspension_entity.end());
    entities.push_back(vehicle.null_con_entity);

    return entities;
}
