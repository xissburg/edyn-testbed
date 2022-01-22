#include <edyn/constraints/soft_distance_constraint.hpp>
#include <edyn/math/scalar.hpp>
#include <entt/entity/registry.hpp>

struct InputComponent {
    bool enabled {false};
    edyn::scalar stiffness, damping;
};

template<typename Archive>
void serialize(Archive &archive, InputComponent &input) {
    archive(input.enabled, input.stiffness, input.damping);
}

inline void UpdateInput(entt::registry &registry) {
    registry.view<InputComponent, edyn::soft_distance_constraint>()
        .each([] (InputComponent &input, edyn::soft_distance_constraint &con) {
            con.stiffness = input.enabled ? input.stiffness : 0;
            con.damping = input.enabled ? input.damping : 0;
        });
}