#ifndef EDYN_TESTBED_PICK_INPUT_HPP
#define EDYN_TESTBED_PICK_INPUT_HPP

#include <edyn/comp/island.hpp>
#include <edyn/util/rigidbody.hpp>
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
    for (auto [entity, position, input] : registry.view<edyn::position, PickInput>().each()) {
        if (position != input.position) {
            position = input.position;
            edyn::wake_up_entity(registry, entity);
        }
    }
}

#endif // EDYN_TESTBED_PICK_INPUT_HPP
