#include "edyn_example.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

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
        edyn::load_mesh_from_obj("/home/xissburg/Documents/Projects/ExhibitionOfSpeed/playground/resources/media/models/racetrack/SmallRacetrack.obj", 
                                 trimesh->vertices, trimesh->indices);

        /* auto extent_x = 2;
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
        } */
        
        /* trimesh->vertices.push_back({0.5, 0, -0.5});
        trimesh->vertices.push_back({-0.5, 0, -0.5});
        trimesh->vertices.push_back({0.5, 0, 0.5});
        trimesh->vertices.push_back({0, 0.5, 1});
        trimesh->vertices.push_back({0, 0.6, -2});

        trimesh->indices.push_back(0);
        trimesh->indices.push_back(1);
        trimesh->indices.push_back(2); */

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

        /* trimesh->is_concave_edge[0] = true;
        trimesh->is_concave_edge[1] = true;
        trimesh->is_concave_edge[2] = true; */

        auto floor_def = edyn::rigidbody_def();
        floor_def.presentation = true;
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 1;
        floor_def.friction = 0.5;
        floor_def.shape_opt = {edyn::mesh_shape{trimesh}};
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
            def.friction = 0.1;
            def.mass = 100;
            def.shape_opt = {edyn::cylinder_shape{0.36, 0.11}};
            def.update_inertia();
            def.restitution = 0;
            def.stiffness = 20000;
            def.damping = 100;
            def.position = {0.25, 1, -0.25};
            def.linvel = {0, 0, 0};
            def.angvel = {0, 0, 0};
            def.orientation = 
                //edyn::quaternion_axis_angle({0,1,0}, edyn::pi * 0.06) *
                //edyn::quaternion_axis_angle({1,0,0}, -edyn::pi * 0.015) *
                edyn::quaternion_axis_angle({0,0,1}, edyn::pi * 0.5);
            auto ent = edyn::make_rigidbody(m_registry, def);
            m_registry.assign<edyn::sleeping_disabled_tag>(ent);
        }

        m_pause = true;
    }
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleTriangleMesh
	, "00-triangle-mesh"
	, "Triangle Mesh."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
	);


