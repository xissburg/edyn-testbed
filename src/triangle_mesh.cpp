#include "edyn_example.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

void ContactStarted(entt::entity ent, entt::registry &reg, edyn::contact_point &cp) {
    auto &rel = reg.get<edyn::relation>(cp.parent);
    auto posA = reg.get<edyn::position>(rel.entity[0]);
    auto ornA = reg.get<edyn::orientation>(rel.entity[0]);
    auto pivot = posA + edyn::rotate(ornA, cp.pivotA);
    edyn::scalar impulse = 0;

    auto *con = reg.try_get<edyn::constraint>(ent);
    if (con) {
        auto &row = reg.get<edyn::constraint_row>(con->row[0]);
        impulse = row.impulse;
    }

    std::cout << "Started | imp: " << impulse << " pos: (" << pivot.x << ", " << pivot.y << ", " << pivot.z << ")" << std::endl;
}

void ContactEnded(entt::entity ent, entt::registry &reg) {
    auto &cp = reg.get<edyn::contact_point>(ent);
    auto &rel = reg.get<edyn::relation>(cp.parent);
    auto posA = reg.get<edyn::position>(rel.entity[0]);
    auto ornA = reg.get<edyn::orientation>(rel.entity[0]);
    auto pivot = posA + edyn::rotate(ornA, cp.pivotA);
    std::cout << "Ended | pos: (" << pivot.x << ", " << pivot.y << ", " << pivot.z << ")" << std::endl;
}

class ExampleTriangleMesh : public EDynExample
{
public:
	ExampleTriangleMesh(const char* _name, const char* _description, const char* _url)
		: EDynExample(_name, _description, _url)
	{

	}

    virtual ~ExampleTriangleMesh() {}

    void createScene() override {
        // Create entities.
        // Create floor
        auto trimesh = std::make_shared<edyn::triangle_mesh>();

    #define LOAD_TRI_MESH 1
    #define PLANAR_TRI_MESH 0

    #if LOAD_TRI_MESH
        auto input = edyn::file_input_archive("SmallRacetrack.bin");

        if (input.is_file_open()) {
            edyn::serialize(input, *trimesh);
        } else {
            edyn::load_mesh_from_obj("/home/xissburg/Documents/Projects/ExhibitionOfSpeed/playground/resources/media/models/racetrack/SmallRacetrack.obj", 
                                    trimesh->vertices, trimesh->indices);
            trimesh->initialize();
            auto output = edyn::file_output_archive("SmallRacetrack.bin");
            edyn::serialize(output, *trimesh);
        }
    #elif PLANAR_TRI_MESH
        auto extent_x = 2;
        auto extent_z = 12;
        auto num_vertices_x = 4;
        auto num_vertices_z = 24;
        edyn::make_plane_mesh(extent_x, extent_z, num_vertices_x, num_vertices_z, 
                              trimesh->vertices, trimesh->indices);

        for (int z = 0; z < num_vertices_z; ++z) {
            auto t = (edyn::scalar(z) / edyn::scalar(num_vertices_z - 1)) * 2 - 1;
            auto y = (t*t*t*t - t*t) * 1.2;
            
            for (int x = 0; x < num_vertices_x; ++x) {
                trimesh->vertices[z * num_vertices_x + x].y = y + (x == 0 || x == num_vertices_x - 1 ? 0.1 : 0);
            }
        }
        trimesh->initialize();
    #else
        trimesh->vertices.push_back({0.5, 0, -0.5});
        trimesh->vertices.push_back({-0.5, 0, -0.5});
        trimesh->vertices.push_back({0.5, 0, 0.5});
        trimesh->vertices.push_back({0, 0.5, 1});
        trimesh->vertices.push_back({0, 0.6, -2});

        trimesh->indices.push_back(0);
        trimesh->indices.push_back(1);
        trimesh->indices.push_back(2);

        /* trimesh->indices.push_back(1);
        trimesh->indices.push_back(4);
        trimesh->indices.push_back(0); */

 /* 
        trimesh->indices.push_back(0);
        trimesh->indices.push_back(2);
        trimesh->indices.push_back(3);

        trimesh->indices.push_back(1);
        trimesh->indices.push_back(4);
        trimesh->indices.push_back(0); */

        trimesh->initialize();
    #endif

        auto *buffer = new edyn::memory_output_archive_source::buffer_type();
        auto output_source = edyn::memory_output_archive_source(*buffer);
        auto paged_trimesh = std::make_shared<edyn::paged_triangle_mesh<edyn::memory_input_archive_source>>(*buffer);
        edyn::load_paged_triangle_mesh(*paged_trimesh, 
                                       trimesh->vertices.begin(), trimesh->vertices.end(),
                                       trimesh->indices.begin(), trimesh->indices.end(),
                                       output_source, 512);

        auto floor_def = edyn::rigidbody_def();
        floor_def.presentation = true;
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 1;
        floor_def.friction = 0.5;
        floor_def.shape_opt = {edyn::paged_mesh_shape{paged_trimesh}};
        edyn::make_rigidbody(m_registry, floor_def);

        // Add some bouncy spheres.
        {
            /* auto def = edyn::rigidbody_def();
            def.presentation = true;
            def.friction = 0.8;
            def.mass = 100;
            def.shape_opt = {edyn::sphere_shape{0.2}};
            def.update_inertia();
            def.restitution = 0.8;
            def.position = {0, 5, 0};
            edyn::make_rigidbody(m_registry, def); */
        /* 
            def.restitution = 0.1;
            def.position = {-0.1, 4, -0.3};
            edyn::make_rigidbody(m_registry, def); */
        }

        {
            auto def = edyn::rigidbody_def();
            def.presentation = true;
            def.friction = 0.7;
            def.mass = 100;
            def.shape_opt = {edyn::cylinder_shape{0.36, 0.11}};
            def.update_inertia();
            def.restitution = 0;
            //def.stiffness = 20000;
            //def.damping = 100;
            def.position = {0.25, 1, -0.25};
            def.linvel = {0, 0, 0};
            def.angvel = {0, 0, 0};
            def.orientation = 
                edyn::quaternion_axis_angle({0,1,0}, edyn::pi * 0.06) *
                edyn::quaternion_axis_angle({1,0,0}, -edyn::pi * 0.015);// *
                //edyn::quaternion_axis_angle({0,0,1}, edyn::pi * 0.5);
            auto ent = edyn::make_rigidbody(m_registry, def);
            m_registry.assign<edyn::sleeping_disabled_tag>(ent);
        }

        /* {
            auto def = edyn::rigidbody_def();
            def.presentation = true;
            def.friction = 0.5;
            def.mass = 80;
            def.shape_opt = {edyn::box_shape{0.26, 0.15, 0.18}};
            def.update_inertia();
            def.restitution = 0;
            //def.stiffness = 20000;
            //def.damping = 100;
            def.position = {0.1, 1.2, -1.1};
            def.linvel = {0, 0, 0};
            def.angvel = {0, 0, 0};
            def.orientation =  edyn::quaternion_axis_angle({0,1,0}, edyn::pi * 0.06);
            auto ent = edyn::make_rigidbody(m_registry, def);
            m_registry.assign<edyn::sleeping_disabled_tag>(ent);
        }

        {
            auto def = edyn::rigidbody_def();
            def.presentation = true;
            def.friction = 0.9;
            def.mass = 60;
            def.shape_opt = {edyn::sphere_shape{0.3}};
            def.update_inertia();
            def.restitution = 0;
            //def.stiffness = 20000;
            //def.damping = 100;
            def.position = {-0.1, 1.4, 0.8};
            def.linvel = {0, 0, 0};
            def.angvel = {0, 0, 0};
            auto ent = edyn::make_rigidbody(m_registry, def);
            m_registry.assign<edyn::sleeping_disabled_tag>(ent);
        } */

        auto &world = m_registry.ctx<edyn::world>();
        world.step_sink().connect<&ExampleTriangleMesh::worldStep>(*this);
        //m_registry.on_construct<edyn::contact_point>().connect<&ContactStarted>();
        m_registry.on_destroy<edyn::contact_point>().connect<&ContactEnded>();

        m_pause = true;
    }

    void worldStep(uint64_t step) {
        auto cp_view = m_registry.view<edyn::contact_point>();
        cp_view.each([&] (entt::entity entity, edyn::contact_point &cp) {
            if (cp.lifetime == 0) {
                ContactStarted(entity, m_registry, cp);
            }
        });
    }
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleTriangleMesh
	, "00-triangle-mesh"
	, "Triangle Mesh."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
	);


