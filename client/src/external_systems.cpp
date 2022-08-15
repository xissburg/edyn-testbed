#include "edyn_example.hpp"
#include <edyn/replication/register_external.hpp>
#include <edyn/util/shape_util.hpp>
#include <random>

struct ClimbBehavior {
    edyn::vector3 direction;
};

// This function is called in the island worker, in a worker thread. The
// provided registry is the private registry of that island worker. Entities
// in this registry do not match up with the entities created in the main
// registry.
void UpdateClimbers(entt::registry &registry) {
    auto manifoldView = registry.view<edyn::contact_manifold>();
    auto climblersView = registry.view<ClimbBehavior, edyn::position>();

    climblersView.each([&](entt::entity entity, ClimbBehavior &climber, edyn::position &posClimber) {
        // Find contact manifolds for this entity.
        edyn::visit_edges(registry, entity, [&](auto manifoldEntity) {
            if (!manifoldView.contains(manifoldEntity)) {
                return;
            }

            auto [manifold] = manifoldView.get(manifoldEntity);

            if (manifold.num_points == 0) {
                return;
            }

            // Only interact with static entities.
            if (manifold.body[0] == entity && !registry.any_of<edyn::static_tag>(manifold.body[1])) {
                return;
            }

            if (manifold.body[1] == entity && !registry.any_of<edyn::static_tag>(manifold.body[0])) {
                return;
            }

            // Find deepest point.
            auto penetration = EDYN_SCALAR_MAX;
            auto pointIndex = edyn::contact_manifold::invalid_id;

            for (size_t i = 0; i < manifold.num_points; ++i) {
                auto &cp = manifold.get_point(i);
                if (cp.distance < penetration) {
                    penetration = cp.distance;
                    pointIndex = i;
                }
            }

            auto &point = manifold.get_point(pointIndex);
            // Calculate direction which goes up.
            edyn::vector3 pivot;

            if (manifold.body[0] == entity) {
                auto pos = registry.get<edyn::position>(manifold.body[1]);
                auto orn = registry.get<edyn::orientation>(manifold.body[1]);
                pivot = edyn::to_world_space(point.pivotB, pos, orn);
            } else {
                EDYN_ASSERT(manifold.body[1] == entity);
                auto pos = registry.get<edyn::position>(manifold.body[0]);
                auto orn = registry.get<edyn::orientation>(manifold.body[0]);
                pivot = edyn::to_world_space(point.pivotA, pos, orn);
            }

            auto normal = point.normal;
            auto dir = -edyn::project_direction(normal, {0, 1, 0});

            if (edyn::try_normalize(dir)) {
                climber.direction = dir;
            }

            // Calculate angular impulse.
            auto axis = edyn::normalize(edyn::cross(normal, climber.direction));
            auto tangentialImpulse = edyn::scalar(1);

            auto maxTangentialImpulse = point.normal_impulse * point.friction;
            tangentialImpulse = std::min(1 + (1 - normal.y) * 33, maxTangentialImpulse);

            auto distance = length(posClimber - pivot);
            auto angularImpulse = axis * tangentialImpulse * distance;

            // Apply impulse.
            auto &inertiaInv = registry.get<edyn::inertia_world_inv>(entity);
            registry.get<edyn::angvel>(entity) += inertiaInv * angularImpulse;
        });
    });
}

class ExampleExternalSystems : public EdynExample
{
public:
    ExampleExternalSystems(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleExternalSystems() {}

    void createScene() override
    {
        edyn::register_external_components<ClimbBehavior>(*m_registry);
        edyn::set_pre_step_callback(*m_registry, &UpdateClimbers);

        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 0;
        floor_def.material->friction = 1.8;

        auto extent_x = 25;
        auto extent_z = 25;
        auto num_vertices_x = 32;
        auto num_vertices_z = 32;
        std::vector<edyn::vector3> vertices;
        std::vector<uint32_t> indices;
        edyn::make_plane_mesh(extent_x, extent_z,
                              num_vertices_x, num_vertices_z,
                              vertices, indices);

        for (int i = 0; i < num_vertices_x; ++i) {
            auto x = (edyn::scalar(i) /edyn::scalar(num_vertices_x) * 2 - 1) * extent_x;
            for (int j = 0; j < num_vertices_z; ++j) {
                auto z = (edyn::scalar(j) /edyn::scalar(num_vertices_z) * 2 - 1) * extent_z;
                vertices[i * num_vertices_x + j].y = (x * x + z * z) * 0.02;
            }
        }

        auto trimesh = std::make_shared<edyn::triangle_mesh>();
        trimesh->insert_vertices(vertices.begin(), vertices.end());
        trimesh->insert_indices(indices.begin(), indices.end());
        trimesh->initialize();

        floor_def.shape = edyn::mesh_shape{trimesh};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Create climbers.
        auto def = edyn::rigidbody_def();
        def.mass = 50;
        def.material->friction = 0.6;
        def.material->restitution = 0;
        def.position = {0, 1, 0};
        def.shape = edyn::sphere_shape{0.2};
        def.update_inertia();

        for (size_t i = 0; i < 10; ++i) {
            def.position = {0, 1.f + edyn::scalar(i) * 0.4f, 0};
            auto climber_entity = edyn::make_rigidbody(*m_registry, def);

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dist(-edyn::pi, edyn::pi);
            auto angle = edyn::scalar(dist(gen));
            auto dir = edyn::vector3{std::cos(angle), 0, std::sin(angle)};
            m_registry->emplace<ClimbBehavior>(climber_entity, dir);
        }
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleExternalSystems
    , "13-external-systems"
    , "External Systems."
    , "https://github.com/xissburg/edyn-testbed"
    );
