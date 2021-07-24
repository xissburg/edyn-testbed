#include "edyn_example.hpp"
#include <iostream>

void ContactStarted(entt::registry &registry, entt::entity entity) {
    auto &imp = registry.get<edyn::constraint_impulse>(entity);
    auto normal_impulse = imp.values[0];

    std::cout << "Started | impulse: " << normal_impulse << std::endl;
}

void ContactEnded(entt::registry &registry, entt::entity entity) {
    auto &cp = registry.get<edyn::contact_point>(entity);
    std::cout << "Ended | lifetime: " << cp.lifetime << std::endl;
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
        floor_def.restitution = 0;
        floor_def.friction = 0.8;

        auto trimesh = std::make_shared<edyn::triangle_mesh>();
        auto input = edyn::file_input_archive("terrain.bin");

        if (input.is_file_open()) {
            edyn::serialize(input, *trimesh);
        } else {
            // If binary is not found, load obj then export binary.
            auto vertices = std::vector<edyn::vector3>{};
            auto indices = std::vector<uint16_t>{};
            auto scale = edyn::scalar(0.05) * edyn::vector3_one;
            edyn::load_tri_mesh_from_obj("../../../edyn-testbed/resources/terrain.obj",
                                         vertices, indices, {0,0,0}, {0,0,0,1}, scale);
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
            def.friction = 0.4;
            def.mass = 50;
            def.restitution = 0;

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
        m_registry->on_construct<edyn::contact_constraint>().connect<&ContactStarted>();
        m_registry->on_destroy<edyn::contact_point>().connect<&ContactEnded>();
    }
};

ENTRY_IMPLEMENT_MAIN(
	ExampleTriangleMesh
	, "10-triangle-mesh"
	, "Triangle Mesh."
    , "https://github.com/xissburg/edyn-testbed"
	);
