#include "pick_input.hpp"
#include <edyn/replication/register_external.hpp>
#include <edyn/networking/networking_external.hpp>

void RegisterNetworkedComponents(entt::registry &registry) {
    edyn::register_external_components<PickInput>(registry);
    edyn::register_networked_components<PickInput>(registry);
}

void UnregisterNetworkedComponents(entt::registry &registry) {
        edyn::remove_external_components(registry);
    edyn::unregister_networked_components(registry);
}
