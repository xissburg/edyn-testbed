#include "edyn_example.hpp"
#include <edyn/util/shape_util.hpp>
#include <edyn/util/shape_io.hpp>

class ExampleRaycasting : public EdynExample
{
public:
    ExampleRaycasting(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleRaycasting() {}

    void createScene() override
    {
        // Create floor
        auto extent_x = 12;
        auto extent_z = 12;
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

        // Add a variety of shapes.
        auto def = edyn::rigidbody_def();
        def.mass = 50;
        def.material->friction = 0.8;
        def.material->restitution = 0;

        auto shapes_and_positions = std::vector<std::pair<edyn::shapes_variant_t, edyn::vector3>>{};

        shapes_and_positions.emplace_back(
            edyn::cylinder_shape{0.15, 0.2, edyn::coordinate_axis::y},
            edyn::vector3{0, 0.2, 0});

        shapes_and_positions.emplace_back(
            edyn::sphere_shape{0.2},
            edyn::vector3{0.5, 1, 0});

        shapes_and_positions.emplace_back(
            edyn::box_shape{0.2, 0.15, 0.25},
            edyn::vector3{1.1, 0.9, 0});

        shapes_and_positions.emplace_back(
            edyn::capsule_shape{0.15, 0.2, edyn::coordinate_axis::z},
            edyn::vector3{1.6, 1, 0});

        shapes_and_positions.emplace_back(
            edyn::load_convex_polyhedrons_from_obj("../../../edyn-testbed/resources/rock.obj",
                                                   {0,0,0}, {0,0,0,1}, {0.8,0.9,1.1}).front().shape,
            edyn::vector3{2.1, 0.9, 0});

        shapes_and_positions.emplace_back(
            edyn::load_compound_shape_from_obj("../../../edyn-testbed/resources/chain_link.obj"),
            edyn::vector3{2.5, 1, 0});

        shapes_and_positions.emplace_back(
            edyn::load_convex_polyhedrons_from_obj("../../../edyn-testbed/resources/cylinder.obj").front().shape,
            edyn::vector3{-0.1, 0.9, 0.5});

        for (auto [shape, pos] : shapes_and_positions) {
            def.position = pos;
            def.shape = shape;
            edyn::make_rigidbody(*m_registry, def);
        }
    }

    void updatePhysics(float deltaTime) override {
        EdynExample::updatePhysics(deltaTime);

        if (m_rayDir == m_prevRayDir) {
            return;
        }

        m_prevRayDir = m_rayDir;
        auto p0 = edyn::vector3{cameraGetPosition().x, cameraGetPosition().y, cameraGetPosition().z};
        auto p1 = p0 + m_rayDir * m_rayLength;

        bool async_execution = edyn::get_execution_mode(*m_registry) == edyn::execution_mode::asynchronous;

        if (async_execution) {
            auto delegate = entt::delegate(entt::connect_arg_t<&ExampleRaycasting::onRaycastResult>{}, *this);
            edyn::raycast_async(*m_registry, p0, p1, delegate);
        } else {
            auto result = edyn::raycast(*m_registry, p0, p1);
            processRaycast(result);
        }
    }

    void processRaycast(const edyn::raycast_result &result) {
        m_registry->clear<edyn::shape_raycast_result>();

        if (result.entity != entt::null) {
            m_registry->emplace<edyn::shape_raycast_result>(result.entity, result);
        }
    }

    void onRaycastResult(edyn::raycast_id_type id, const edyn::raycast_result &result,
                         edyn::vector3 p0, edyn::vector3 p1) {
        processRaycast(result);
    }

private:
    edyn::vector3 m_prevRayDir;
};

ENTRY_IMPLEMENT_MAIN(
    ExampleRaycasting
    ,"15-raycasting"
    , "Ray-casting."
    , "https://github.com/xissburg/edyn-testbed"
    );
