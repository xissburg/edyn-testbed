#include "edyn_example.hpp"
#include <edyn/replication/register_external.hpp>
#include <edyn/util/aabb_util.hpp>
#include <edyn/util/shape_util.hpp>

struct HoverForce {
    struct RayForce {
        edyn::vector3 loc;
        edyn::vector3 dir;
        edyn::scalar stiffness;
        edyn::scalar damping;
        edyn::scalar length;
        edyn::scalar radius;
    };

    static constexpr uint8_t max_count = 4;
    uint8_t count;
    std::array<RayForce, max_count> rays;
};

void ApplyRayForces(entt::registry &registry) {
    auto dt = edyn::get_fixed_dt(registry);
    auto vel_view = registry.view<edyn::linvel, edyn::angvel>();
    auto tr_view = registry.view<edyn::position, edyn::orientation>();
    auto view = registry.view<HoverForce, edyn::position, edyn::orientation>().each();
    auto dyn_view = registry.view<edyn::dynamic_tag>();

    for (auto [entity, coll, pos, orn] : view) {
        for (int i = 0; i < coll.count; ++i) {
            auto &ray = coll.rays[i];
            auto dir = rotate(orn, ray.dir);

            // Use a set of 4 springs per corner. Cast a ray for each.
            constexpr auto num_springs = 4;
            std::array<edyn::vector3, num_springs> ray_locs;
            std::array<edyn::raycast_result, num_springs> results;
            std::vector<entt::entity> ignore;
            ignore.push_back(entity);
            int count = 0;

            for (int j = 0; j < num_springs; ++j) {
                auto ray_loc_offset_unit = edyn::vector3{edyn::scalar(j % 2 == 0 ? 1 : -1), 0,
                                                         edyn::scalar(j < 2 ? 1 : -1)};
                auto ray_loc = ray.loc + ray_loc_offset_unit * ray.radius;
                auto p0 = edyn::to_world_space(ray_loc, pos, orn);
                auto p1 = p0 + dir * ray.length;
                auto res = edyn::raycast(registry, p0, p1, {entity});

                if (res.entity != entt::null) {
                    results[count] = res;
                    ray_locs[count] = p0;
                    ++count;
                }
            }

            // Only apply forces for rays that intersected something.
            for (int j = 0; j < count; ++j) {
                auto &res = results[j];
                auto p0 = ray_locs[j];

                auto inclination_factor = std::abs(edyn::dot(res.normal, dir));
                auto spring_force = (edyn::scalar(1) - res.fraction) * ray.length * ray.stiffness * inclination_factor * res.normal;
                edyn::vector3 velA = {0,0,0}, velB = {0,0,0};

                if (vel_view.contains(entity)) {
                    velA = vel_view.get<edyn::linvel>(entity) + edyn::cross(vel_view.get<edyn::angvel>(entity), p0 - pos);
                }

                if (vel_view.contains(res.entity)) {
                    velB = vel_view.get<edyn::linvel>(res.entity) +
                           edyn::cross(vel_view.get<edyn::angvel>(res.entity), p0 - tr_view.get<edyn::position>(res.entity));
                }

                auto rel_vel = velA - velB;
                auto normal_rel_vel = res.normal * edyn::dot(-rel_vel, res.normal);
                auto damping_force = normal_rel_vel * ray.damping;

                // Divide impulse by the number of rays that intersected for
                // correct force distribution.
                auto impulse = (spring_force + damping_force) * dt / count;
                edyn::rigidbody_apply_impulse(registry, entity, impulse, p0 - pos);

                if (dyn_view.contains(res.entity)) {
                    auto pos_other = tr_view.get<edyn::position>(res.entity);
                    edyn::rigidbody_apply_impulse(registry, res.entity, -impulse, p0 - pos_other);
                }
            }
        }
    }
}

class ExampleHover : public EdynExample
{
public:
    ExampleHover(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleHover() {}

    void createScene() override
    {
        edyn::register_external_components<HoverForce>(*m_registry);
        edyn::set_pre_step_callback(*m_registry, &ApplyRayForces);

        // Create floor
        auto extent_x = 25;
        auto extent_z = 25;
        auto num_vertices_x = 32;
        auto num_vertices_z = 32;
        std::vector<edyn::vector3> vertices;
        std::vector<uint32_t> indices;
        edyn::make_plane_mesh(extent_x, extent_z,
                              num_vertices_x, num_vertices_z,
                              vertices, indices);

        // Make a slight bowl shape.
        for (int i = 0; i < num_vertices_x; ++i) {
            auto x = (edyn::scalar(i) /edyn::scalar(num_vertices_x) * 2 - 1) * extent_x;
            for (int j = 0; j < num_vertices_z; ++j) {
                auto z = (edyn::scalar(j) /edyn::scalar(num_vertices_z) * 2 - 1) * extent_z;
                vertices[i * num_vertices_x + j].y = (x * x + z * z) * 0.002;
            }
        }

        auto trimesh = std::make_shared<edyn::triangle_mesh>();
        trimesh->insert_vertices(vertices.begin(), vertices.end());
        trimesh->insert_indices(indices.begin(), indices.end());
        trimesh->initialize();

        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 0;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::mesh_shape{trimesh};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add a box to act as hover.
        auto hover_def = edyn::rigidbody_def();
        hover_def.mass = 500;
        hover_def.material->friction = 0.4;
        hover_def.material->restitution = 0;
        auto box = edyn::box_shape{0.6, 0.4, 1.2};
        hover_def.shape = box;
        hover_def.position = {-10, 2.2, 0};
        hover_def.orientation = edyn::quaternion_axis_angle({0, 1, 0}, edyn::half_pi);
        auto hover = edyn::make_rigidbody(*m_registry, hover_def);

        // Create force components, one for each corner of the box.
        auto &hover_force = m_registry->emplace<HoverForce>(hover);
        hover_force.count = 4;

        for (int i = 0; i < hover_force.count; ++i) {
            auto &ray = hover_force.rays[i];
            ray.dir = {0, -1, 0};
            ray.length = 0.5;
            ray.stiffness = 8000;
            ray.damping = 400;
            ray.loc = box.half_extents * edyn::vector3{edyn::scalar(i % 2 == 0 ? 1 : -1),
                                                       edyn::scalar(-1),
                                                       edyn::scalar(i < 2 ? 1 : -1)};
            ray.radius = 0.15;
        }
        m_registry->patch<HoverForce>(hover);

        for (int i = 0; i < 4; ++i) {
            auto def = edyn::rigidbody_def{};
            def.mass = 100;
            def.material->friction = 0.7;
            def.material->restitution = 0;
            def.shape = edyn::box_shape{0.2, 0.15, 0.8};
            def.position = {edyn::scalar(-4 + i * 2), 0.2, 0};
            edyn::make_rigidbody(*m_registry, def);
        }
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleHover
    , "28-hover"
    , "Hover with raycasts."
    , "https://github.com/xissburg/edyn-testbed"
    );
