#include <edyn/comp/tag.hpp>
#include <edyn/networking/comp/entity_owner.hpp>
#include <edyn/networking/comp/remote_client.hpp>
#include <entt/entity/registry.hpp>
#include <edyn/edyn.hpp>
#include <edyn/networking/networking.hpp>
#include "vehicle_system.hpp"
#include "edyn_server.hpp"
#include "pick_input.hpp"

void create_scene(entt::registry &registry) {
    // Create floor.
    auto floor_def = edyn::rigidbody_def();
    floor_def.kind = edyn::rigidbody_kind::rb_static;
    floor_def.material->restitution = 0.3;
    floor_def.material->friction = 1;
    floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
    floor_def.networked = true;
    edyn::make_rigidbody(registry, floor_def);
}

void on_construct_remote_client(entt::registry &registry, entt::entity client_entity) {
    auto vehicle_entity = CreateVehicle(registry);
    std::vector<entt::entity> entities;
    entities.push_back(vehicle_entity);

    auto &vehicle = registry.get<Vehicle>(vehicle_entity);
    entities.push_back(vehicle.chassis_entity);
    entities.insert(entities.end(), vehicle.wheel_entity.begin(), vehicle.wheel_entity.end());
    entities.insert(entities.end(), vehicle.suspension_entity.begin(), vehicle.suspension_entity.end());
    entities.push_back(vehicle.null_con_entity);

    auto &client = registry.get<edyn::remote_client>(client_entity);
    client.owned_entities.insert(client.owned_entities.end(), entities.begin(), entities.end());

    for (auto entity : entities) {
        registry.emplace<edyn::entity_owner>(entity, client_entity);
        registry.emplace<edyn::networked_tag>(entity);
    }
}

void ExternalSystemUpdate(entt::registry &registry) {
    UpdatePickInput(registry);
    UpdateVehicles(registry);
}

int main() {
    entt::registry registry;

    edyn_server_init(registry);

    edyn::register_external_components<
        PickInput,
        Vehicle,
        VehicleSettings,
        VehicleState,
        VehicleInput
    >(registry);

    edyn::register_networked_components<
        PickInput,
        Vehicle,
        VehicleSettings,
        VehicleState,
        VehicleInput
    >(registry, std::tuple<VehicleState, PickInput>{}, std::tuple<VehicleInput, PickInput>{});

    edyn::set_external_system_pre_step(registry, &ExternalSystemUpdate);

    registry.on_construct<edyn::remote_client>().connect<&on_construct_remote_client>();

    create_scene(registry);

    edyn_server_run(registry);

    edyn_server_deinit(registry);

    return 0;
}
