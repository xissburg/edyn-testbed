#ifndef EDYNTESTBED_EDYN_SERVER_HPP
#define EDYNTESTBED_EDYN_SERVER_HPP

#include <unordered_map>
#include <entt/entity/fwd.hpp>
#include <enet/enet.h>
#include <edyn/networking/networking.hpp>

bool edyn_server_init(entt::registry &registry);
void edyn_server_deinit(entt::registry &registry);
void edyn_server_run(entt::registry &registry);
void edyn_server_update(entt::registry &registry);

#endif // EDYNTESTBED_EDYN_SERVER_HPP
