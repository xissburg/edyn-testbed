#include <edyn/replication/register_external.hpp>
#include <edyn/util/rigidbody.hpp>
#include <entt/entity/registry.hpp>
#include <edyn/networking/networking.hpp>
#include "pick_input.hpp"
#include "edyn_server.hpp"
#include "server_ports.hpp"

void create_scene(entt::registry &registry) {
    // Create floor.
    auto floor_def = edyn::rigidbody_def();
    floor_def.kind = edyn::rigidbody_kind::rb_static;
    floor_def.material->restitution = 0;
    floor_def.material->friction = 0.5;
    floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
    floor_def.networked = true;
    floor_def.presentation = false;
    edyn::make_rigidbody(registry, floor_def);

    // Create boxes.
    auto def = edyn::rigidbody_def();
    def.mass = 10;
    def.material->friction = 0.8;
    def.material->restitution = 0;
    def.shape = edyn::box_shape{0.2, 0.2, 0.2};
    def.networked = true;
    def.presentation = false;

    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                def.position = {edyn::scalar(0.4 * j),
                                edyn::scalar(0.4 * i + 0.6),
                                edyn::scalar(0.4 * k)};
                edyn::make_rigidbody(registry, def);
            }
        }
    }
}

void edyn_server_update(entt::registry &registry) {}

int main() {
    entt::registry registry;

    edyn_server_init(registry, NetworkingServerPort);

    edyn::register_external_components<PickInput>(registry);
    edyn::register_networked_components<PickInput>(registry);
    edyn::set_pre_step_callback(registry, &UpdatePickInput);

    create_scene(registry);

    edyn_server_run(registry);

    edyn_server_deinit(registry);

    return 0;
}
