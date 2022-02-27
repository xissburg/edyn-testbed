#ifndef EDYNTESTBED_EDYN_SERVER_HPP
#define EDYNTESTBED_EDYN_SERVER_HPP

#include <unordered_map>
#include <entt/entity/fwd.hpp>
#include <enet/enet.h>
#include <edyn/networking/networking.hpp>

static constexpr uint16_t NetworkingServerPort = 1337;
static constexpr uint16_t VehicleServerPort = 1338;

bool edyn_server_init(entt::registry &registry, uint16_t port);
void edyn_server_deinit(entt::registry &registry);
void edyn_server_run(entt::registry &registry);
void edyn_server_update(entt::registry &registry);

#endif // EDYNTESTBED_EDYN_SERVER_HPP
