#include "edyn_example.hpp"
#include <edyn/collision/contact_point.hpp>
#include <edyn/serialization/paged_triangle_mesh_s11n.hpp>
#include <edyn/shapes/create_paged_triangle_mesh.hpp>
#include <edyn/util/shape_io.hpp>
#include <edyn/util/paged_mesh_load_reporting.hpp>
#include <iostream>

void PageLoaded(entt::registry &registry, entt::entity entity, unsigned index) {
    auto &mesh = registry.get<edyn::paged_mesh_shape>(entity);
    auto trimesh = mesh.trimesh->get_submesh(index);

    if (trimesh) {
        std::cout << "Page " << index << " loaded: "
                << trimesh->num_vertices() << " verts, "
                << trimesh->num_edges() << " edges, "
                << trimesh->num_triangles() << " tris."
                << std::endl;
    } else {
        std::cout << "Unloaded page " << index << std::endl;
    }
}

class ExamplePagedTriangleMesh : public EdynExample
{
public:
    ExamplePagedTriangleMesh(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

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

    virtual ~ExamplePagedTriangleMesh() {}

    void createScene() override {
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 0;
        floor_def.material->friction = 0.8;

        m_input = std::make_shared<edyn::paged_triangle_mesh_file_input_archive>("terrain_large.bin", edyn::get_enqueue_task(*m_registry));
        auto paged_trimesh = std::make_shared<edyn::paged_triangle_mesh>(std::static_pointer_cast<edyn::triangle_mesh_page_loader_base>(m_input));
        paged_trimesh->m_max_cache_num_vertices = 1 << 14;

        if (m_input->is_file_open()) {
            edyn::serialize(*m_input, *paged_trimesh);
        } else {
            // Load a large mesh, split it into smaller submeshes, write them to files
            // and setup the paged triangle mesh to read them on demand.
            std::vector<edyn::vector3> vertices;
            std::vector<uint32_t> indices;
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
                1 << 11, {}, {}, edyn::get_enqueue_task_wait(*m_registry));

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

        floor_def.shape = edyn::paged_mesh_shape{paged_trimesh};
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
        m_registry->on_construct<edyn::contact_point>().connect<&ExamplePagedTriangleMesh::contactStarted>(*this);
        m_registry->on_destroy<edyn::contact_point>().connect<&ExamplePagedTriangleMesh::contactPointDestroyed>(*this);

        edyn::on_paged_mesh_page_loaded(*m_registry).connect<&PageLoaded>(*m_registry);
    }

    void destroyScene() override {
        m_input->close();
        m_input.reset();
    }

    void updatePhysics(float deltaTime) override {
        EdynExample::updatePhysics(deltaTime);
        processNewContacts();
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
