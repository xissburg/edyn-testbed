#ifndef EDYN_TESTBED_VEHICLE_SYSTEM_HPP
#define EDYN_TESTBED_VEHICLE_SYSTEM_HPP

#include <array>
#include <vector>
#include <variant>
#include <edyn/math/math.hpp>
#include <edyn/math/scalar.hpp>
#include <entt/entity/fwd.hpp>
#include <edyn/util/entity_map.hpp>
#include <edyn/comp/merge_component.hpp>

struct Vehicle {
    entt::entity chassis_entity;
    std::array<entt::entity, 4> wheel_entity;
    std::array<entt::entity, 4> suspension_entity;
    entt::entity null_con_entity;
};

struct VehicleInput {
    edyn::scalar steering{};
    edyn::scalar throttle{};
    edyn::scalar brakes{};
};

struct VehicleState {
    edyn::scalar target_steering;
    edyn::scalar steering;
    std::array<edyn::scalar, 4> throttle;
    std::array<edyn::scalar, 4> brakes;
};

struct VehicleSettings {
    edyn::scalar max_steering_angle {edyn::to_radians(45)};
    edyn::scalar max_steering_rate {edyn::to_radians(180)};
    edyn::scalar brake_torque {edyn::scalar(800)};
    edyn::scalar bearing_torque {edyn::scalar(6)};
    edyn::scalar driving_torque {edyn::scalar(300)};
};

template<typename Archive>
void serialize(Archive &archive, Vehicle &vehicle) {
    archive(vehicle.chassis_entity);
    archive(vehicle.wheel_entity);
    archive(vehicle.suspension_entity);
    archive(vehicle.null_con_entity);
}

template<typename Archive>
void serialize(Archive &archive, VehicleInput &input) {
    archive(input.steering);
    archive(input.throttle);
    archive(input.brakes);
}

template<typename Archive>
void serialize(Archive &archive, VehicleState &state) {
    archive(state.target_steering);
    archive(state.steering);
    archive(state.throttle);
    archive(state.brakes);
}

template<typename Archive>
void serialize(Archive &archive, VehicleSettings &settings) {
    archive(settings.max_steering_angle);
    archive(settings.max_steering_rate);
    archive(settings.brake_torque);
    archive(settings.bearing_torque);
    archive(settings.driving_torque);
}

struct VehicleSteeringAction {
    edyn::scalar value {};
};

struct VehicleThrottleAction {
    edyn::scalar value {};
};

struct VehicleBrakeAction {
    edyn::scalar value {};
};

using VehicleActionVariant = std::variant<VehicleSteeringAction, VehicleBrakeAction, VehicleThrottleAction>;
using VehicleActionsVector = std::vector<VehicleActionVariant>;

struct VehicleActions {
    VehicleActionsVector values;

    auto begin() {
        return values.begin();
    }

    auto end() {
        return values.end();
    }

    auto & operator[](size_t i) {
        return values[i];
    }

    void clear() {
        values.clear();
    }

    void push_back(const VehicleActionVariant &value) {
        values.push_back(value);
    }
};

template<typename Archive>
void serialize(Archive &archive, VehicleActions &actions) {
    archive(actions.values);
}

// Declare custom merge function to accumulate actions instead of replace.
template<>
void edyn::merge_component<VehicleActions>(VehicleActions &actions, const VehicleActions &new_value);

void RegisterVehicleComponents(entt::registry &);
entt::entity CreateVehicle(entt::registry &);
void UpdateVehicles(entt::registry &);
std::vector<entt::entity> GetVehicleEntities(entt::registry &, entt::entity);

#endif // EDYN_TESTBED_VEHICLE_SYSTEM_HPP
