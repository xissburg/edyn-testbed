#ifndef EDYN_TESTBED_VEHICLE_SYSTEM_HPP
#define EDYN_TESTBED_VEHICLE_SYSTEM_HPP

#include <array>
#include <entt/core/hashed_string.hpp>
#include <map>
#include <vector>
#include <variant>
#include <edyn/math/math.hpp>
#include <edyn/math/scalar.hpp>
#include <entt/entity/fwd.hpp>

struct Vehicle {
    entt::entity chassis_entity;
    std::array<entt::entity, 4> wheel_entity;
    std::array<entt::entity, 4> suspension_entity;
    entt::entity null_con_entity;
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
    edyn::scalar camber {edyn::to_radians(-6)};
};

template<typename Archive>
void serialize(Archive &archive, Vehicle &vehicle) {
    archive(vehicle.chassis_entity);
    archive(vehicle.wheel_entity);
    archive(vehicle.suspension_entity);
    archive(vehicle.null_con_entity);
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
    archive(settings.camber);
}

inline void merge_component(VehicleState &state, const VehicleState &new_value) {
    // Always select the steering value that's closer to the target
    // to avoid glitches due to overriding, particularly after
    // extrapolation.
    const auto steering_remaining = std::abs(state.steering - new_value.target_steering);
    const auto steering_remaining_new = std::abs(new_value.steering - new_value.target_steering);
    const auto steering = steering_remaining < steering_remaining_new ? state.steering : new_value.steering;
    state = new_value;
    state.steering = steering;
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

struct VehicleAction {
    std::variant<VehicleSteeringAction, VehicleBrakeAction, VehicleThrottleAction> var;
};

template<typename Archive>
void serialize(Archive &archive, VehicleSteeringAction &action) {
    archive(action.value);
}

template<typename Archive>
void serialize(Archive &archive, VehicleThrottleAction &action) {
    archive(action.value);
}

template<typename Archive>
void serialize(Archive &archive, VehicleBrakeAction &action) {
    archive(action.value);
}

template<typename Archive>
void serialize(Archive &archive, VehicleAction &action) {
    archive(action.var);
}

enum class VehicleAssetEntry : unsigned short {
    Vehicle,
    Chassis,
    NullCon,
    WheelFL,
    WheelFR,
    WheelRL,
    WheelRR,
    SuspensionFL,
    SuspensionFR,
    SuspensionRL,
    SuspensionRR,
};
static constexpr auto VehicleAssetID = entt::hashed_string{"VehicleAsset"};

void RegisterVehicleComponents(entt::registry &);
void RegisterNetworkedVehicleComponents(entt::registry &);
entt::entity CreateVehicle(entt::registry &);
entt::entity MakeVehicleNetworked(entt::registry &, entt::entity vehicleEntity);
void UpdateVehicles(entt::registry &);
std::vector<entt::entity> GetVehicleEntities(entt::registry &, entt::entity);

std::map<entt::id_type, entt::entity>
CreateVehicleAssetEntityMap(entt::registry &registry, entt::entity vehicleEntity);

#endif // EDYN_TESTBED_VEHICLE_SYSTEM_HPP
