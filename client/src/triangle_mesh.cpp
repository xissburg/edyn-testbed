#include "edyn_example.hpp"
#include <edyn/collision/contact_manifold.hpp>
#include <edyn/edyn.hpp>
#include <iostream>

void ContactStarted(entt::registry &registry, entt::entity entity) {
    auto &manifold = registry.get<edyn::contact_manifold>(entity);
    EDYN_ASSERT(manifold.num_points > 0);
    float normal_impulse = 0;

    manifold.each_point([&](edyn::contact_point &cp) {
        normal_impulse += cp.normal_impulse + cp.normal_restitution_impulse;
    });

    std::cout << "Started | impulse: " << normal_impulse << std::endl;
}

void ContactPointDestroyed(entt::registry &registry, entt::entity entity,
                           edyn::contact_manifold::contact_id_type cp_id) {
    auto &manifold = registry.get<edyn::contact_manifold>(entity);
    auto lifetime = manifold.point[cp_id].lifetime;
    std::cout << "Ended | lifetime: " << lifetime << std::endl;
}

class ExampleTriangleMesh : public EdynExample
{
public:
    ExampleTriangleMesh(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleTriangleMesh() {}

    void createScene() override {
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 0;
        floor_def.material->friction = 0.8;

        auto trimesh = std::make_shared<edyn::triangle_mesh>();
        auto input = edyn::file_input_archive("terrain.bin");

        if (input.is_file_open()) {
            edyn::serialize(input, *trimesh);
        } else {
            // If binary is not found, load obj then export binary.
            auto vertices = std::vector<edyn::vector3>{};
            auto indices = std::vector<uint32_t>{};
            auto scale = edyn::scalar(0.1) * edyn::vector3_one;
            edyn::load_tri_mesh_from_obj("../../../edyn-testbed/resources/terrain.obj",
                                         vertices, indices, nullptr, {0,0,0}, {0,0,0,1}, scale);
            trimesh->insert_vertices(vertices.begin(), vertices.end());
            trimesh->insert_indices(indices.begin(), indices.end());
            trimesh->initialize();
            auto output = edyn::file_output_archive("terrain.bin");
            edyn::serialize(output, *trimesh);
        }

        floor_def.shape = edyn::mesh_shape{trimesh};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some dynamic entities.
        {
            auto def = edyn::rigidbody_def();
            def.mass = 50;
            def.material->friction = 0.4;
            def.material->restitution = 0;

            auto shapes_and_positions = std::vector<std::pair<edyn::shapes_variant_t, edyn::vector3>>{};

            shapes_and_positions.emplace_back(
                edyn::cylinder_shape{0.15, 0.2},
                edyn::vector3{0, 1, 0});

            shapes_and_positions.emplace_back(
                edyn::sphere_shape{0.2},
                edyn::vector3{0.5, 1, 0});

            shapes_and_positions.emplace_back(
                edyn::box_shape{0.2, 0.15, 0.25},
                edyn::vector3{1.1, 0.9, 0});

            shapes_and_positions.emplace_back(
                edyn::capsule_shape{0.15, 0.2},
                edyn::vector3{1.6, 1, 0});

            shapes_and_positions.emplace_back(
                edyn::polyhedron_shape("../../../edyn-testbed/resources/rock.obj", {0,0,0}, {0,0,0,1}, {0.8,0.9,1.1}),
                edyn::vector3{2.1, 0.9, 0});

            shapes_and_positions.emplace_back(
                edyn::compound_shape("../../../edyn-testbed/resources/chain_link.obj"),
                edyn::vector3{2.5, 1, 0});

            for (auto [shape, pos] : shapes_and_positions) {
                def.position = pos;
                def.shape = shape;
                def.update_inertia();
                edyn::make_rigidbody(*m_registry, def);
            }
        }

        // Collision events example.
        edyn::on_contact_started(*m_registry).connect<&ContactStarted>(*m_registry);
        //edyn::on_contact_ended(*m_registry).connect<&ContactEnded>(*m_registry);
        edyn::on_contact_point_destroyed(*m_registry).connect<&ContactPointDestroyed>(*m_registry);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleTriangleMesh
    , "10-triangle-mesh"
    , "Triangle Mesh."
    , "https://github.com/xissburg/edyn-testbed"
    );
