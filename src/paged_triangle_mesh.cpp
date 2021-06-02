#include "edyn_example.hpp"

void ContactStarted(entt::registry &registry, entt::entity entity);
void ContactEnded(entt::registry &registry, entt::entity entity);

class ExamplePagedTriangleMesh : public EdynExample
{
public:
	ExamplePagedTriangleMesh(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
        , m_input(std::make_shared<edyn::paged_triangle_mesh_file_input_archive>())
	{

	}

    virtual ~ExamplePagedTriangleMesh() {}

    void createScene() override {
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 0;
        floor_def.friction = 0.8;

        auto paged_trimesh = std::make_shared<edyn::paged_triangle_mesh>(std::static_pointer_cast<edyn::triangle_mesh_page_loader_base>(m_input));
        paged_trimesh->m_max_cache_num_vertices = 1 << 14;
        m_input->open("terrain_large.bin");

        if (m_input->is_file_open()) {
            edyn::serialize(*m_input, *paged_trimesh);
        } else {
            // Load a large mesh, split it into smaller submeshes, write them to files
            // and setup the paged triangle mesh to read them on demand.
            std::vector<edyn::vector3> vertices;
            std::vector<uint16_t> indices;
            // Note: the working directory is bgfx/examples/runtime and it is
            // assumed the edyn-testbed directory is at the same level.
            auto obj_path = "../../../edyn-testbed/resources/terrain_large.obj";
            edyn::load_tri_mesh_from_obj(obj_path, vertices, indices);

            // Generate triangle mesh from .obj file. This splits the mesh into
            // a bunch of smaller `triangle_mesh` which are stored in the
            // `paged_triangle_mesh` nodes.
            edyn::create_paged_triangle_mesh(
                *paged_trimesh,
                vertices.begin(), vertices.end(),
                indices.begin(), indices.end(),
                1 << 11);

            {
                // After creating the paged triangle mesh all nodes are loaded into
                // the cache, then it's the best time to write them all to files.
                // This scope is to ensure the file is commited to external storage
                // before reading from it.
                auto output = edyn::paged_triangle_mesh_file_output_archive("terrain_large.bin",
                    edyn::paged_triangle_mesh_serialization_mode::external);
                edyn::serialize(output, *paged_trimesh);
                paged_trimesh->clear_cache();
            }

            // Now load it from file so the `paged_triangle_mesh_file_input_archive`
            // knows where to load submeshes from.
            m_input->open("terrain_large.bin");
            edyn::serialize(*m_input, *paged_trimesh);
        }

        floor_def.shape_opt = {edyn::paged_mesh_shape{paged_trimesh}};
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
                def.shape_opt = {shape};
                def.update_inertia();
                edyn::make_rigidbody(*m_registry, def);
            }
        }

        // Collision events example.
        m_registry->on_construct<edyn::constraint_impulse>().connect<&ContactStarted>();
        m_registry->on_destroy<edyn::contact_point>().connect<&ContactEnded>();
    }

private:
    std::shared_ptr<edyn::paged_triangle_mesh_file_input_archive> m_input;
};

ENTRY_IMPLEMENT_MAIN(
	ExamplePagedTriangleMesh
	, "11-paged-triangle-mesh"
	, "Paged Triangle Mesh."
    , "https://github.com/xissburg/edyn-testbed"
	);
