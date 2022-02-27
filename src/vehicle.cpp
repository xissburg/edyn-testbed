#include "edyn_example.hpp"
#include "vehicle_system.hpp"
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
        floor_def.material->friction = 1;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        m_vehicle_entity = CreateVehicle(*m_registry);
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
};

ENTRY_IMPLEMENT_MAIN(
    ExampleVehicle
    , "23-vehicle"
    , "Basic multi-body vehicle."
    , "https://github.com/xissburg/edyn"
    );
