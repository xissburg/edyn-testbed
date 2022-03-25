#include "pick_input.hpp"
#include <entt/entity/registry.hpp>
#include <edyn/networking/networking_external.hpp>

void RegisterNetworkedComponents(entt::registry &registry) {
    edyn::register_external_components<PickInput>(registry);
    edyn::register_networked_components<PickInput>(registry, std::tuple<PickInput>{}, std::tuple<PickInput>{});
}

void UnregisterNetworkedComponents(entt::registry &registry) {
    edyn::remove_external_components(registry);
    edyn::unregister_networked_components(registry);
}
