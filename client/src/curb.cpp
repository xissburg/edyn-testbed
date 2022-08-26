#include "edyn_example.hpp"
#include <edyn/util/shape_util.hpp>

class ExampleCurb : public EdynExample
{
public:
    ExampleCurb(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleCurb() {}

    void createScene() override
    {
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

        for (int i = 0; i < num_vertices_x; ++i) {
            for (int j = 0; j < num_vertices_z; ++j) {
                auto z = (edyn::scalar(j) /edyn::scalar(num_vertices_z) * 2 - 1) * extent_z;

                if (z > 0) {
                    auto &v = vertices[i * num_vertices_x + j];
                    v.x -= float(extent_x) / (num_vertices_x - 1);
                    v.y = 0.16;
                }
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

        auto def = edyn::rigidbody_def();
        def.mass = 40;
        def.material->friction = 0.8;
        def.material->restitution = 0;
        def.material->roll_friction = 0.005;
        def.shape = edyn::cylinder_shape{0.3, 0.2, edyn::coordinate_axis::x};
        def.update_inertia();

        edyn::make_rigidbody(*m_registry, def);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleCurb
    , "25-curb"
    , "Curb."
    , "https://github.com/xissburg/edyn-testbed"
    );
