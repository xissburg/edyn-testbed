#include "edyn_example.hpp"
#include "vehicle.hpp"
#include <edyn/comp/orientation.hpp>
#include <edyn/constraints/generic_constraint.hpp>
#include <edyn/constraints/null_constraint.hpp>
#include <edyn/edyn.hpp>
#include <edyn/math/math.hpp>
#include <edyn/math/quaternion.hpp>
#include <edyn/math/vector3.hpp>
#include <edyn/util/constraint_util.hpp>
#include <edyn/util/rigidbody.hpp>

class ExampleVehicle : public EdynExample
{
public:
    ExampleVehicle(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    void createScene() override
    {
        edyn::register_external_components<Vehicle, VehicleSettings, VehicleState, VehicleInput>(*m_registry);
        edyn::set_external_system_pre_step(*m_registry, &UpdateVehicles);

        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 0.3;
        floor_def.material->friction = 1.1;

        m_input = std::make_shared<edyn::paged_triangle_mesh_file_input_archive>();
        auto paged_trimesh = std::make_shared<edyn::paged_triangle_mesh>(std::static_pointer_cast<edyn::triangle_mesh_page_loader_base>(m_input));
        paged_trimesh->m_max_cache_num_vertices = 1 << 14;
        m_input->open("terrain_large.bin");

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
                1 << 11, {});

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

        m_vehicle_entity = m_registry->create();
        edyn::tag_external_entity(*m_registry, m_vehicle_entity);

        auto &vehicle = m_registry->emplace<Vehicle>(m_vehicle_entity);

        // Vehicle body.
        auto chassis_def = edyn::rigidbody_def();
        chassis_def.material->restitution = 0.3;
        chassis_def.material->friction = 0.3;
        chassis_def.mass = 600;
        chassis_def.shape = edyn::box_shape{0.65, 0.5, 1.85};
        chassis_def.update_inertia();
        chassis_def.continuous_contacts = true;
        chassis_def.position = {0, 2, 0};
        auto chassis_entity = edyn::make_rigidbody(*m_registry, chassis_def);
        vehicle.chassis_entity = chassis_entity;

        edyn::make_constraint<edyn::null_constraint>(*m_registry, m_vehicle_entity, chassis_entity);

        // Wheels.
        auto wheel_def = edyn::rigidbody_def{};
        wheel_def.material->restitution = 0.6;
        wheel_def.material->friction = 1;
        wheel_def.mass = 50;
        wheel_def.shape = edyn::cylinder_shape{0.4, 0.1};
        wheel_def.update_inertia();

        for (int i = 0; i < 4; ++i) {
            auto lateral = i == 0 || i == 2 ? 1 : -1;
            auto longitudinal = i == 0 || i == 1 ? 1 : -1;
            wheel_def.position = {lateral * 0.9f, 1.5, longitudinal * 1.45f};
            auto wheel_entity = edyn::make_rigidbody(*m_registry, wheel_def);

            edyn::exclude_collision(*m_registry, chassis_entity, wheel_entity);

            auto [con_ent, con] = edyn::make_constraint<edyn::generic_constraint>(*m_registry, chassis_entity, wheel_entity);
            con.pivot[0] = {lateral * 0.8f, 0, longitudinal * 1.45f};
            con.pivot[1] = {-0.1f * lateral, 0, 0};
            con.linear_dofs[1].offset_min = -0.8;
            con.linear_dofs[1].offset_max = 0;
            con.linear_dofs[1].bump_stop_length = 0.1;
            con.linear_dofs[1].bump_stop_stiffness = 250000;
            con.linear_dofs[1].spring_stiffness = 55000;
            con.linear_dofs[1].rest_offset = -0.5;
            con.linear_dofs[1].damping = 2000;
            con.linear_dofs[1].friction_force = 50;
            con.angular_dofs[0].limit_enabled = false;
            con.angular_dofs[0].friction_torque = edyn::to_Nm_per_radian(0.01);

            vehicle.wheel_entity[i] = wheel_entity;
            vehicle.suspension_entity[i] = con_ent;
        }

        m_registry->emplace<VehicleSettings>(m_vehicle_entity);
        m_registry->emplace<VehicleInput>(m_vehicle_entity);
        m_registry->emplace<VehicleState>(m_vehicle_entity);
    }

    void destroyScene() override {
        EdynExample::destroyScene();
        edyn::remove_external_components(*m_registry);
        edyn::remove_external_systems(*m_registry);
    }

    void setSteering(float steering) {
        auto &input = m_registry->get<VehicleInput>(m_vehicle_entity);
        input.steering = steering;
        edyn::refresh<VehicleInput>(*m_registry, m_vehicle_entity);
    }

    void setThrottle(float throttle) {
        auto &input = m_registry->get<VehicleInput>(m_vehicle_entity);
        input.throttle = throttle;
        edyn::refresh<VehicleInput>(*m_registry, m_vehicle_entity);
    }

    void setBrakes(float brakes) {
        auto &input = m_registry->get<VehicleInput>(m_vehicle_entity);
        input.brakes = brakes;
        edyn::refresh<VehicleInput>(*m_registry, m_vehicle_entity);
    }

    void updatePhysics(float deltaTime) override {
        EdynExample::updatePhysics(deltaTime);

        if (inputGetKeyState(entry::Key::Left)) {
            setSteering(-1);
        } else if (inputGetKeyState(entry::Key::Right)) {
            setSteering(1);
        } else {
            setSteering(0);
        }

        if (inputGetKeyState(entry::Key::Up)) {
            setThrottle(1);
        } else {
            setThrottle(0);
        }

        if (inputGetKeyState(entry::Key::Down)) {
            setBrakes(1);
        } else {
            setBrakes(0);
        }
    }

    entt::entity m_vehicle_entity;
    std::shared_ptr<edyn::paged_triangle_mesh_file_input_archive> m_input;
};

ENTRY_IMPLEMENT_MAIN(
    ExampleVehicle
    , "24-vehicle"
    , "Basic multi-body vehicle."
    , "https://github.com/xissburg/edyn"
    );

void UpdateVehicles(entt::registry &registry) {
    auto vehicle_view = registry.view<const Vehicle, const VehicleInput, const VehicleSettings, VehicleState>().each();
    auto dt = edyn::get_fixed_dt(registry);

    // Apply vehicle input and update state.
    for (auto [entity, vehicle, input, settings, state] : vehicle_view) {
        // Update steering.
        state.target_steering = input.steering * settings.max_steering_angle;
        auto steering_direction = state.target_steering > state.steering ? 1 : -1;
        auto steering_increment = std::min(settings.max_steering_rate * dt, std::abs(state.target_steering - state.steering)) * steering_direction;
        state.steering += steering_increment;

        // Update brakes and traction with rudimentary ABS and TCS.
        for (int i = 0; i < 4; ++i) {
            auto wheel_entity = vehicle.wheel_entity[i];
            auto &wheel_linvel = registry.get<edyn::linvel>(wheel_entity);
            auto &wheel_angvel = registry.get<edyn::angvel>(wheel_entity);
            auto &wheel_orn = registry.get<edyn::orientation>(wheel_entity);
            auto spin_axis = edyn::quaternion_x(wheel_orn);
            auto spin_speed = edyn::dot(wheel_angvel, spin_axis);
            auto longitudinal_speed = edyn::length(edyn::project_direction(wheel_linvel, spin_axis));

            if (longitudinal_speed > 2 &&
                std::abs(spin_speed) < edyn::to_radians(1))
            {
                state.brakes[i] = 0;
            } else {
                state.brakes[i] = input.brakes;
            }

            if (std::abs(spin_speed) * 0.25f > longitudinal_speed) {
                state.throttle[i] = 0;
            } else {
                state.throttle[i] = input.throttle;
            }
        }
    }

    // Apply vehicle state.
    for (auto [entity, vehicle, settings, state] : registry.view<const Vehicle, const VehicleSettings, const VehicleState>().each()) {
        for (int i = 0; i < 2; ++i) {
            auto steering = state.steering;

            // Slight Ackerman effect.
            if ((i == 0 && steering > 0) || (i == 1 && steering < 0)) {
                steering *= 1.1;
            }

            auto &con = registry.get<edyn::generic_constraint>(vehicle.suspension_entity[i]);
            con.frame[0] = edyn::to_matrix3x3(edyn::quaternion_axis_angle({0, 1, 0}, steering));
        }

        for (int i = 0; i < 4; ++i) {
            auto &con = registry.get<edyn::generic_constraint>(vehicle.suspension_entity[i]);
            con.angular_dofs[0].friction_torque = state.brakes[i] * settings.brake_torque + settings.bearing_torque;

            auto wheel_entity = vehicle.wheel_entity[i];
            auto &wheel_orn = registry.get<edyn::orientation>(wheel_entity);
            auto spin_axis = edyn::quaternion_x(wheel_orn);
            auto driving_torque = state.throttle[i] * settings.driving_torque * spin_axis;
            edyn::rigidbody_apply_torque_impulse(registry, vehicle.wheel_entity[i], driving_torque * dt);
        }
    }
}
