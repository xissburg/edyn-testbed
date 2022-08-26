#include "networking.hpp"
#include "server_ports.hpp"
#include "vehicle_system.hpp"
#include "pick_input.hpp"

void PreStepUpdate(entt::registry &registry) {
    UpdatePickInput(registry);
    UpdateVehicles(registry);
}

class ExampleVehicleNetworking : public ExampleNetworking
{
public:
    ExampleVehicleNetworking(const char* _name, const char* _description, const char* _url)
        : ExampleNetworking(_name, _description, _url)
    {
        m_server_port = VehicleServerPort;
    }

    void onConstructVehicle(entt::registry &registry, entt::entity entity) {
        if (edyn::client_owns_entity(registry, entity)) {
            m_vehicle_entity = entity;
        }
    }

    void createScene() override
    {
        m_vehicle_entity = entt::null;

        ExampleNetworking::createScene();

        RegisterNetworkedVehicleComponents(*m_registry);
        RegisterVehicleComponents(*m_registry);

        edyn::set_pre_step_callback(*m_registry, &PreStepUpdate);

        m_registry->on_construct<Vehicle>().connect<&ExampleVehicleNetworking::onConstructVehicle>(*this);
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
        if (m_vehicle_entity != entt::null) {
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

        ExampleNetworking::updatePhysics(deltaTime);
    }

    entt::entity m_vehicle_entity{entt::null};
    edyn::scalar m_steering{};
    edyn::scalar m_throttle{};
    edyn::scalar m_brakes{};
};

ENTRY_IMPLEMENT_MAIN(
    ExampleVehicleNetworking
    , "27-vehicle-networking"
    , "Networked vehicle."
    , "https://github.com/xissburg/edyn"
    );
