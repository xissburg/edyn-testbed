#ifndef EDYN_TESTBED_VEHICLE_HPP
#define EDYN_TESTBED_VEHICLE_HPP

#include <array>
#include <edyn/math/math.hpp>
#include <edyn/math/scalar.hpp>
#include <entt/entity/fwd.hpp>
#include <edyn/parallel/merge/merge_component.hpp>
#include <edyn/util/entity_map.hpp>

struct Vehicle {
    entt::entity chassis_entity;
    std::array<entt::entity, 4> wheel_entity;
    std::array<entt::entity, 4> suspension_entity;
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

namespace edyn {
    template<> inline
    void merge(Vehicle &new_comp, const entity_map &emap) {
        new_comp.chassis_entity = emap.remloc(new_comp.chassis_entity);

        for (auto &entity : new_comp.wheel_entity) {
            entity = emap.remloc(entity);
        }

        for (auto &entity : new_comp.suspension_entity) {
            entity = emap.remloc(entity);
        }
    }
}

void UpdateVehicles(entt::registry &);

#endif // EDYN_TESTBED_VEHICLE_HPP
