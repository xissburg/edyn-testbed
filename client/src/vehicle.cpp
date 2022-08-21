#include "edyn_example.hpp"
#include "vehicle_system.hpp"
#include <edyn/comp/action_list.hpp>
#include <edyn/replication/register_external.hpp>

class ExampleVehicle : public EdynExample
{
public:
    ExampleVehicle(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    void createScene() override
    {
        if (edyn::get_execution_mode(*m_registry) == edyn::execution_mode::asynchronous) {
            RegisterVehicleComponents(*m_registry);
        }

        edyn::set_pre_step_callback(*m_registry, &UpdateVehicles);

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
        edyn::remove_pre_step_callback(*m_registry);
    }

    using ActionList = edyn::action_list<VehicleAction>;

    void insertAction(VehicleAction action) {
        if (!m_registry->all_of<ActionList>(m_vehicle_entity)) {
            m_registry->emplace<ActionList>(m_vehicle_entity);
        }

        m_registry->patch<ActionList>(m_vehicle_entity, [&](ActionList &list) {
            list.actions.push_back(action);
        });
    }

    void setSteering(float steering) {
        if (m_steering != steering) {
            m_steering = steering;
            insertAction(VehicleAction{VehicleSteeringAction{steering}});
        }
    }

    void setThrottle(float throttle) {
        if (m_throttle != throttle) {
            m_throttle = throttle;
            insertAction(VehicleAction{VehicleThrottleAction{throttle}});
        }
    }

    void setBrakes(float brakes) {
        if (m_brakes != brakes) {
            m_brakes = brakes;
            insertAction(VehicleAction{VehicleBrakeAction{brakes}});
        }
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
    edyn::scalar m_steering{};
    edyn::scalar m_throttle{};
    edyn::scalar m_brakes{};
};

ENTRY_IMPLEMENT_MAIN(
    ExampleVehicle
    , "24-vehicle"
    , "Basic multi-body vehicle."
    , "https://github.com/xissburg/edyn"
    );
