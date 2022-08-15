#include "edyn_example.hpp"
#include <edyn/util/shape_util.hpp>

class ExamplePerVertexMaterials : public EdynExample
{
public:
    ExamplePerVertexMaterials(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExamplePerVertexMaterials() {}

    void createScene() override
    {
        // Create floor
        auto vertices = std::vector<edyn::vector3>{};
        auto colors = std::vector<edyn::vector3>{};
        auto indices = std::vector<uint32_t>{};
        auto scale = edyn::scalar(1) * edyn::vector3_one;
        auto rotation = edyn::quaternion_axis_angle({1,0,0}, -edyn::half_pi);
        edyn::load_tri_mesh_from_obj("../../../edyn-testbed/resources/plane_per_vert.obj",
                                     vertices, indices, &colors, {0,0,0}, rotation, scale);

        // Get friction and restitution from vertex colors.
        EDYN_ASSERT(!colors.empty());
        auto friction = std::vector<edyn::scalar>{};
        auto restitution = std::vector<edyn::scalar>{};

        for (auto color : colors) {
            friction.push_back(color.x);
            restitution.push_back(color.y);
        }

        auto trimesh = std::make_shared<edyn::triangle_mesh>();
        trimesh->insert_vertices(vertices.begin(), vertices.end());
        trimesh->insert_indices(indices.begin(), indices.end());
        trimesh->insert_friction_coefficients(friction.begin(), friction.end());
        trimesh->insert_restitution_coefficients(restitution.begin(), restitution.end());
        trimesh->initialize();

        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.shape = edyn::mesh_shape{trimesh};
        // Restitution of the trimesh must be non-zero or else the restitution
        // solver would ignore this rigid body. More precisely, a
        // `edyn::contact_manifold_with_restitution` tag would not be assigned
        // to contact manifolds containing this body. The exact value is not used
        // because the per-vertex restitution will be considered instead.
        floor_def.material->restitution = 1;
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add boxes.
        auto def = edyn::rigidbody_def();
        def.mass = 10;
        def.material->friction = 0.8;
        def.material->restitution = 0.7;
        def.shape = edyn::box_shape{0.2, 0.2, 0.2};
        def.update_inertia();
        def.continuous_contacts = true;

        for (int i = 0; i < 14; ++i) {
            for (int j = 0; j < 1; ++j) {
                for (int k = 0; k < 1; ++k) {
                    def.position = {edyn::scalar(0.4 * j),
                                    edyn::scalar(0.4 * i + 0.6),
                                    edyn::scalar(0.4 * k)};
                    edyn::make_rigidbody(*m_registry, def);
                }
            }
        }
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExamplePerVertexMaterials
    , "18-per-vertex-materials"
    , "Per-vertex materials."
    , "https://github.com/xissburg/edyn-testbed"
    );
