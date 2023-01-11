#ifndef EDYN_TESTBED_PICK_INPUT_HPP
#define EDYN_TESTBED_PICK_INPUT_HPP

#include <edyn/comp/island.hpp>
#include <edyn/util/island_util.hpp>
#include <entt/entity/registry.hpp>
#include <edyn/comp/position.hpp>
#include <edyn/networking/comp/network_input.hpp>

struct PickInput : edyn::network_input {
    edyn::vector3 position;
};

template<typename Archive>
void serialize(Archive &archive, PickInput &input) {
    archive(input.position);
}

inline void UpdatePickInput(entt::registry &registry) {
    auto resident_view = registry.view<edyn::multi_island_resident>();

    for (auto [entity, position, input] : registry.view<edyn::position, PickInput>().each()) {
        if (position != input.position) {
            position = input.position;

            auto [resident] = resident_view.get(entity);
            edyn::wake_up_island(registry, *resident.island_entities.begin());
        }
    }
}

#endif // EDYN_TESTBED_PICK_INPUT_HPP
