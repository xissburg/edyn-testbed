#include "edyn_example.hpp"
#include <edyn/comp/tag.hpp>
#include <edyn/dynamics/moment_of_inertia.hpp>
#include <edyn/shapes/shapes.hpp>
#include <edyn/util/rigidbody.hpp>
#include <edyn/util/shape_io.hpp>
#include <edyn/util/shape_util.hpp>
#include <random>

class ExampleShapeShifting : public EdynExample
{
public:
    ExampleShapeShifting(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleShapeShifting() {}

    void createScene() override
    {
        m_shapes.push_back(edyn::sphere_shape{0.2});
        m_shapes.push_back(edyn::box_shape{0.2, 0.2, 0.2});
        m_shapes.push_back(edyn::cylinder_shape{0.2, 0.2, edyn::coordinate_axis::y});
        m_shapes.push_back(edyn::load_convex_polyhedrons_from_obj("../../../edyn-testbed/resources/box.obj").front().shape);
        m_shapes.push_back(edyn::load_convex_polyhedrons_from_obj("../../../edyn-testbed/resources/rock.obj").front().shape);
        m_shapes.push_back(edyn::load_convex_polyhedrons_from_obj("../../../edyn-testbed/resources/cylinder.obj").front().shape);

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

        // Start with boxes.
        auto def = edyn::rigidbody_def();
        def.mass = 10;
        def.material->friction = 0.8;
        def.material->restitution = 0;
        def.shape = edyn::box_shape{0.2, 0.2, 0.2};

        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                for (int k = 0; k < 5; ++k) {
                    def.position = {edyn::scalar(0.4 * j),
                                    edyn::scalar(0.4 * i + 0.6),
                                    edyn::scalar(0.4 * k)};
                    edyn::make_rigidbody(*m_registry, def);
                }
            }
        }
    }

    void updatePhysics(float deltaTime) override
    {
        m_timer += deltaTime;

        if (m_timer > 3) {
            m_timer = 0;

            std::random_device rand_dev;
            std::mt19937 generator(rand_dev());
            std::uniform_int_distribution<int> distr(0, m_shapes.size() - 1);
            auto &shape = m_shapes[distr(generator)];
            auto inertia = edyn::moment_of_inertia(shape, 10);

            for (auto entity : m_registry->view<edyn::dynamic_tag>()) {
                edyn::rigidbody_set_shape(*m_registry, entity, shape);
                edyn::set_rigidbody_inertia(*m_registry, entity, inertia);
            }
        }

        EdynExample::updatePhysics(deltaTime);
    }

    float m_timer {};
    std::vector<edyn::shapes_variant_t> m_shapes;
};

ENTRY_IMPLEMENT_MAIN(
    ExampleShapeShifting
    , "29-shape-shifting"
    , "Shape shifting."
    , "https://github.com/xissburg/edyn-testbed"
    );
