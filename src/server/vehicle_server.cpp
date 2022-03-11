#include <edyn/comp/tag.hpp>
#include <edyn/networking/comp/entity_owner.hpp>
#include <edyn/networking/comp/remote_client.hpp>
#include <entt/entity/registry.hpp>
#include <edyn/edyn.hpp>
#include <edyn/networking/networking.hpp>
#include "vehicle_system.hpp"
#include "edyn_server.hpp"
#include "pick_input.hpp"

std::vector<entt::entity> g_pending_new_clients;

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
    g_pending_new_clients.push_back(client_entity);
}

void assign_vehicle_ownership_to_client(entt::registry &registry,
                                        entt::entity vehicle_entity,
                                        entt::entity client_entity) {
    auto entities = GetVehicleEntities(registry, vehicle_entity);
    auto &client = registry.get<edyn::remote_client>(client_entity);
    client.owned_entities.insert(client.owned_entities.end(), entities.begin(), entities.end());

    for (auto entity : entities) {
        registry.emplace<edyn::entity_owner>(entity, client_entity);
        registry.emplace<edyn::networked_tag>(entity);
    }
}

void notify_vehicle_entity_created(entt::registry &registry,
                                   entt::entity vehicle_entity,
                                   entt::entity client_entity) {
    auto entities = GetVehicleEntities(registry, vehicle_entity);
    edyn::server_notify_created_entities(registry, client_entity, entities);
}

void edyn_server_update(entt::registry &registry) {
    for (auto client_entity : g_pending_new_clients) {
        auto vehicle_entity = CreateVehicle(registry);
        assign_vehicle_ownership_to_client(registry, vehicle_entity, client_entity);
        notify_vehicle_entity_created(registry, vehicle_entity, client_entity);
    }

    g_pending_new_clients.clear();
}

void ExternalSystemUpdate(entt::registry &registry) {
    UpdatePickInput(registry);
    UpdateVehicles(registry);
}

int main() {
    entt::registry registry;

    edyn_server_init(registry, VehicleServerPort);

    edyn::register_networked_components<
        PickInput,
        Vehicle,
        VehicleSettings,
        VehicleState,
        VehicleInput
    >(registry, std::tuple<VehicleState, PickInput>{}, std::tuple<VehicleInput, PickInput>{});

    RegisterVehicleComponents(registry);
    edyn::set_external_system_pre_step(registry, &ExternalSystemUpdate);

    registry.on_construct<edyn::remote_client>().connect<&on_construct_remote_client>();

    create_scene(registry);

    edyn_server_run(registry);

    edyn_server_deinit(registry);

    return 0;
}
