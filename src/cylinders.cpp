#include "edyn_example.hpp"

class ExampleCylinders : public EdynExample
{
public:
	ExampleCylinders(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

    virtual ~ExampleCylinders() {}

	void createScene() override
	{
        // Create floor
        auto extent_x = 25;
        auto extent_z = 25;
        auto num_vertices_x = 32;
        auto num_vertices_z = 32;
        std::vector<edyn::vector3> vertices;
        std::vector<uint16_t> indices;
        edyn::make_plane_mesh(extent_x, extent_z,
                              num_vertices_x, num_vertices_z,
                              vertices, indices);

        // Make a slight bowl shape.
        for (int i = 0; i < num_vertices_x; ++i) {
            auto x = (edyn::scalar(i) /edyn::scalar(num_vertices_x) * 2 - 1) * extent_x;
            for (int j = 0; j < num_vertices_z; ++j) {
                auto z = (edyn::scalar(j) /edyn::scalar(num_vertices_z) * 2 - 1) * extent_z;
                vertices[i * num_vertices_x + j].y = (x * x + z * z) * 0.001;
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

        // Add many cylinders.
        auto def = edyn::rigidbody_def();
        def.mass = 10;
        def.material->friction = 0.8;
        def.material->restitution = 0;
        def.material->roll_friction = 0.005;
        def.shape = edyn::cylinder_shape{0.2, 0.2};
        def.update_inertia();
        def.continuous_contacts = true;
        def.orientation = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi / 2);

        std::vector<edyn::rigidbody_def> defs;

        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                for (int k = 0; k < 5; ++k) {
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
	ExampleCylinders
	, "03-cylinders"
	, "Cylinders."
    , "https://github.com/xissburg/edyn-testbed"
	);
