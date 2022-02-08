#ifndef EDYN_TESTBED_PICK_INPUT_HPP
#define EDYN_TESTBED_PICK_INPUT_HPP

#include <edyn/comp/dirty.hpp>
#include <edyn/edyn.hpp>
#include <entt/entity/registry.hpp>
#include <edyn/math/vector3.hpp>
#include <edyn/comp/position.hpp>

struct PickInput {
    edyn::vector3 position;
};

template<typename Archive>
void serialize(Archive &archive, PickInput &input) {
    archive(input.position);
}

inline void UpdatePickInput(entt::registry &registry) {
    for (auto [entity, position, input] : registry.view<edyn::position, PickInput>().each()) {
        position = input.position;
    }
}

#endif // EDYN_TESTBED_PICK_INPUT_HPP
