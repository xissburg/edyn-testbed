#include "edyn_example.hpp"
#include <edyn/collision/raycast.hpp>

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
        auto extent_x = 12;
        auto extent_z = 12;
        auto num_vertices_x = 32;
        auto num_vertices_z = 32;
        std::vector<edyn::vector3> vertices;
        std::vector<uint16_t> indices;
        std::vector<edyn::scalar> friction_coefficients;
        edyn::make_plane_mesh(extent_x, extent_z,
                              num_vertices_x, num_vertices_z,
                              vertices, indices);
        friction_coefficients.resize(vertices.size());

        // Make a slight bowl shape.
        for (int i = 0; i < num_vertices_x; ++i) {
            auto fraction_x = edyn::scalar(i) /edyn::scalar(num_vertices_x) * 2 - 1;
            auto x = fraction_x * extent_x;
            for (int j = 0; j < num_vertices_z; ++j) {
                auto fraction_z = edyn::scalar(j) /edyn::scalar(num_vertices_z) * 2 - 1;
                auto z = fraction_z * extent_z;
                vertices[i * num_vertices_x + j].y = (x * x + z * z) * 0.001;
                friction_coefficients[i * num_vertices_x + j] = fraction_x * fraction_x + fraction_z * fraction_z;
            }
        }

        auto trimesh = std::make_shared<edyn::triangle_mesh>();
        trimesh->insert_vertices(vertices.begin(), vertices.end());
        trimesh->insert_indices(indices.begin(), indices.end());
        trimesh->insert_friction_coefficients(friction_coefficients.begin(), friction_coefficients.end());
        trimesh->initialize();

        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 0;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::mesh_shape{trimesh};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add boxes.
        auto def = edyn::rigidbody_def();
        def.mass = 10;
        def.material->friction = 0.8;
        def.material->restitution = 0;
        def.shape = edyn::box_shape{0.2, 0.2, 0.2};
        def.update_inertia();
        def.continuous_contacts = true;

        std::vector<edyn::rigidbody_def> defs;

        for (int i = 0; i < 1; ++i) {
            for (int j = 0; j < 1; ++j) {
                for (int k = 0; k < 1; ++k) {
                    def.position = {edyn::scalar(0.4 * j),
                                    edyn::scalar(0.4 * i + 0.6),
                                    edyn::scalar(0.4 * k)};
                    defs.push_back(def);
                }
            }
        }

        edyn::batch_rigidbodies(*m_registry, defs);
	}
};

ENTRY_IMPLEMENT_MAIN(
	ExamplePerVertexMaterials
	, "18-per-vertex-materials"
	, "Per-vertex materials."
    , "https://github.com/xissburg/edyn-testbed"
    );
