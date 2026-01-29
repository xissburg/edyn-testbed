#include "edyn_example.hpp"
#include <edyn/collision/contact_point.hpp>
#include <edyn/serialization/file_archive.hpp>
#include <edyn/util/contact_manifold_util.hpp>
#include <edyn/util/shape_io.hpp>
#include <iostream>

class ExampleTriangleMesh : public EdynExample
{
public:
    ExampleTriangleMesh(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleTriangleMesh() {}

    std::vector<entt::entity> m_newContactEntities;

    void contactStarted(entt::entity entity) {
        m_newContactEntities.push_back(entity);
    }

    void processNewContacts() {
        auto &registry = *m_registry;

        for (auto entity : m_newContactEntities) {
            if (!registry.valid(entity)) continue;

            auto &cp_imp = registry.get<edyn::contact_point_impulse>(entity);
            auto normal_impulse = cp_imp.normal_impulse + cp_imp.normal_restitution_impulse;
            std::cout << "Started | impulse: " << normal_impulse << std::endl;
        }

        m_newContactEntities.clear();
    }

    void contactPointDestroyed(entt::registry &registry, entt::entity entity) {
        auto &cp = registry.get<edyn::contact_point>(entity);
        auto lifetime = cp.lifetime;
        std::cout << "Ended | lifetime: " << lifetime << std::endl;
    }

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
                edyn::cylinder_shape{0.15, 0.2, edyn::coordinate_axis::z},
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
                edyn::load_convex_polyhedrons_from_obj("../../../edyn-testbed/resources/rock.obj",
                                                       {0,0,0}, {0,0,0,1}, {0.8,0.9,1.1}).front().shape,
                edyn::vector3{2.1, 0.9, 0});

            shapes_and_positions.emplace_back(
                edyn::load_compound_shape_from_obj("../../../edyn-testbed/resources/chain_link.obj"),
                edyn::vector3{2.5, 1, 0});

            for (auto [shape, pos] : shapes_and_positions) {
                def.position = pos;
                def.shape = shape;
                edyn::make_rigidbody(*m_registry, def);
            }
        }

        // Collision events example.
        m_registry->on_construct<edyn::contact_point>().connect<&ExampleTriangleMesh::contactStarted>(*this);
        m_registry->on_destroy<edyn::contact_point>().connect<&ExampleTriangleMesh::contactPointDestroyed>(*this);
    }

    void updatePhysics(float deltaTime) override {
        EdynExample::updatePhysics(deltaTime);
        processNewContacts();
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleTriangleMesh
    , "10-triangle-mesh"
    , "Triangle Mesh."
    , "https://github.com/xissburg/edyn-testbed"
    );
