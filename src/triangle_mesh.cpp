#include "edyn_example.hpp"
#include <edyn/math/matrix3x3.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

void ContactStarted(entt::registry &registry, entt::entity entity) {
    if (!registry.has<edyn::contact_point>(entity)) return;

    auto &cp = registry.get<edyn::contact_point>(entity);
    auto posA = registry.get<edyn::position>(cp.body[0]);
    auto ornA = registry.get<edyn::orientation>(cp.body[0]);
    auto pivot = edyn::to_world_space(cp.pivotA, posA, ornA);

    auto &imp = registry.get<edyn::constraint_impulse>(entity);
    auto normal_impulse = imp.values[0];

    std::cout << "Started | imp: " << normal_impulse << " pos: (" << pivot.x << ", " << pivot.y << ", " << pivot.z << ")" << std::endl;
}

void ContactEnded(entt::registry &registry, entt::entity entity) {
    auto &cp = registry.get<edyn::contact_point>(entity);
    auto posA = registry.get<edyn::position>(cp.body[0]);
    auto ornA = registry.get<edyn::orientation>(cp.body[0]);
    auto pivot = posA + edyn::rotate(ornA, cp.pivotA);
    std::cout << "Ended | pos: (" << pivot.x << ", " << pivot.y << ", " << pivot.z << ")" << std::endl;
}

class ExampleTriangleMesh : public EdynExample
{
public:
	ExampleTriangleMesh(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
        , m_input(std::make_shared<edyn::paged_triangle_mesh_file_input_archive>())
	{

	}

    virtual ~ExampleTriangleMesh() {}

    void createScene() override {
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 1;
        floor_def.friction = 0.5;

    #define LOAD_TRI_MESH 1
    #define LOAD_PAGED_TRI_MESH 2
    #define PLANAR_TRI_MESH 3
    #define MANUAL_TRI_MESH 4

    #define MESH_TYPE PLANAR_TRI_MESH

    #if MESH_TYPE == LOAD_TRI_MESH
        auto trimesh = std::make_shared<edyn::triangle_mesh>();
        auto input = edyn::file_input_archive("terrain.bin");

        if (input.is_file_open()) {
            edyn::serialize(input, *trimesh);
        } else {
            // If binary is not found, load obj then export binary.
            edyn::load_tri_mesh_from_obj("../../../edyn-testbed/resources/terrain.obj", 
                                         trimesh->vertices, trimesh->indices);
            trimesh->initialize();
            auto output = edyn::file_output_archive("terrain.bin");
            edyn::serialize(output, *trimesh);
        }
        floor_def.shape_opt = {edyn::mesh_shape{trimesh}};
    #elif MESH_TYPE == PLANAR_TRI_MESH
        auto trimesh = std::make_shared<edyn::triangle_mesh>();
        auto extent_x = 2;
        auto extent_z = 12;
        auto num_vertices_x = 4;
        auto num_vertices_z = 24;
        edyn::make_plane_mesh(extent_x, extent_z, num_vertices_x, num_vertices_z, 
                              trimesh->vertices, trimesh->indices);

        // Make it not so planar.
        for (int z = 0; z < num_vertices_z; ++z) {
            auto t = (edyn::scalar(z) / edyn::scalar(num_vertices_z - 1)) * 2 - 1;
            auto y = (t*t*t*t - t*t) * 1.2;
            
            for (int x = 0; x < num_vertices_x; ++x) {
                trimesh->vertices[z * num_vertices_x + x].y = y + (x == 0 || x == num_vertices_x - 1 ? 0.1 : 0);
            }
        }
        trimesh->initialize();

        floor_def.shape_opt = {edyn::mesh_shape{trimesh}};
    #elif MESH_TYPE == MANUAL_TRI_MESH
        auto trimesh = std::make_shared<edyn::triangle_mesh>();
        trimesh->vertices.push_back({2, 0, 2});
        trimesh->vertices.push_back({2, 0.1, -2});
        trimesh->vertices.push_back({-2, 0, -2});
        trimesh->vertices.push_back({-1, -0.8, 1});

        trimesh->indices.push_back(0);
        trimesh->indices.push_back(1);
        trimesh->indices.push_back(2);

        trimesh->indices.push_back(0);
        trimesh->indices.push_back(2);
        trimesh->indices.push_back(3);

        trimesh->initialize();

        floor_def.shape_opt = {edyn::mesh_shape{trimesh}};
    #elif MESH_TYPE == LOAD_PAGED_TRI_MESH
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
    #else
        static_assert(false, "Invalid mesh type.")
    #endif

        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some dynamic entities.
        {
            auto def = edyn::rigidbody_def();
            def.friction = 0.3;
            def.mass = 100;
            def.restitution = 0.5;
            def.position = {0, 5, 0};

            const size_t n = 1;
            for (size_t i = 0; i < n; ++i) {
                if (i % 2 == 0) {
                    def.shape_opt = {edyn::capsule_shape{0.15, 0.2}};
                } else {
                    //def.shape_opt = {edyn::sphere_shape{0.2}};
                    auto obj_path = "../../../edyn-testbed/resources/cylinder.obj";
                    def.shape_opt = {edyn::polyhedron_shape(obj_path)};
                }

                def.update_inertia();
                def.position = {0, edyn::scalar(0.8 + i * 0.7), 0};
                edyn::make_rigidbody(*m_registry, def);
            }
        }

        // Collision events example.
        m_registry->on_construct<edyn::constraint_impulse>().connect<&ContactStarted>();
        m_registry->on_destroy<edyn::contact_point>().connect<&ContactEnded>();

        m_registry->on_construct<edyn::contact_point>().connect<&EdynExample::onConstructContactPoint>(*this);

        m_pause = true;
        auto& world = m_registry->ctx<edyn::world>();
        world.set_paused(m_pause);
    }

    std::shared_ptr<edyn::paged_triangle_mesh_file_input_archive> m_input;
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleTriangleMesh
	, "03-triangle-mesh"
	, "Triangle Mesh."
	, "https://github.com/xissburg/edyn-testbed"
	);


