#include <edyn/comp/tag.hpp>
#include <edyn/networking/comp/entity_owner.hpp>
#include <edyn/networking/comp/remote_client.hpp>
#include <edyn/networking/comp/aabb_of_interest.hpp>
#include <edyn/networking/comp/aabb_oi_follow.hpp>
#include <edyn/networking/sys/server_side.hpp>
#include <edyn/networking/util/asset_util.hpp>
#include <entt/entity/registry.hpp>
#include <edyn/edyn.hpp>
#include <edyn/networking/networking.hpp>
#include "vehicle_system.hpp"
#include "edyn_server.hpp"
#include "pick_input.hpp"
#include "server_ports.hpp"

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
    client.owned_entities.insert(entities.begin(), entities.end());

    for (auto entity : entities) {
        registry.emplace<edyn::entity_owner>(entity, client_entity);
    }
}

void edyn_server_update(entt::registry &registry) {
    for (auto client_entity : g_pending_new_clients) {
        auto vehicle_entity = CreateVehicle(registry);
        auto asset_entity = MakeVehicleNetworked(registry, vehicle_entity);
        assign_vehicle_ownership_to_client(registry, vehicle_entity, client_entity);

        // Also assign ownership of asset.
        registry.emplace<edyn::entity_owner>(asset_entity, client_entity);
        registry.get<edyn::remote_client>(client_entity).owned_entities.emplace(asset_entity);

        // Make AABB of interest follow vehicle.
        registry.get<edyn::aabb_of_interest>(client_entity).aabb = {-50 * edyn::vector3_one, 50 * edyn::vector3_one};

        auto &veh = registry.get<Vehicle>(vehicle_entity);
        registry.emplace<edyn::aabb_oi_follow>(client_entity, veh.chassis_entity);
    }

    g_pending_new_clients.clear();
}

void PreStepUpdate(entt::registry &registry) {
    UpdatePickInput(registry);
    UpdateVehicles(registry);
}

int main() {
    entt::registry registry;

    edyn_server_init(registry, VehicleServerPort);

    edyn::set_fixed_dt(registry, 0.008);
    edyn::set_solver_velocity_iterations(registry, 16);

    RegisterVehicleComponents(registry);
    RegisterNetworkedVehicleComponents(registry);
    edyn::set_pre_step_callback(registry, &PreStepUpdate);

    registry.on_construct<edyn::remote_client>().connect<&on_construct_remote_client>();

    create_scene(registry);

    edyn_server_run(registry);

    edyn_server_deinit(registry);

    return 0;
}
