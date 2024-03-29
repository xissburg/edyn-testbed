#ifndef EDYNTESTBED_EDYN_SERVER_HPP
#define EDYNTESTBED_EDYN_SERVER_HPP

#include <entt/entity/fwd.hpp>

bool edyn_server_init(entt::registry &registry, uint16_t port);
void edyn_server_deinit(entt::registry &registry);
void edyn_server_run(entt::registry &registry);
void edyn_server_update(entt::registry &registry);

#endif // EDYNTESTBED_EDYN_SERVER_HPP
