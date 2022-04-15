#include "basic_networking.hpp"
#include "server_ports.hpp"
#include "vehicle_system.hpp"
#include <edyn/comp/dirty.hpp>
#include <edyn/comp/orientation.hpp>
#include <edyn/edyn.hpp>
#include <edyn/math/math.hpp>
#include <edyn/math/quaternion.hpp>
#include <edyn/math/vector3.hpp>
#include <edyn/util/constraint_util.hpp>
#include <edyn/util/rigidbody.hpp>
#include "pick_input.hpp"

void ExternalSystemUpdate(entt::registry &registry) {
    UpdatePickInput(registry);
    UpdateVehicles(registry);
}

class ExampleVehicleNetworking : public ExampleBasicNetworking
{
public:
    ExampleVehicleNetworking(const char* _name, const char* _description, const char* _url)
        : ExampleBasicNetworking(_name, _description, _url)
    {
        m_server_port = VehicleServerPort;
    }

    void onConstructVehicle(entt::registry &registry, entt::entity entity) {
        if (edyn::client_owns_entity(registry, entity)) {
            m_vehicle_entity = entity;
            registry.emplace<VehicleInput>(entity);
            registry.emplace<edyn::action_list<VehicleAction>>(entity);
        }
    }

    void createScene() override
    {
        m_vehicle_entity = entt::null;

        ExampleBasicNetworking::createScene();

        RegisterNetworkedVehicleComponents(*m_registry);
        RegisterVehicleComponents(*m_registry);

        edyn::set_external_system_pre_step(*m_registry, &ExternalSystemUpdate);

        m_registry->on_construct<Vehicle>().connect<&ExampleVehicleNetworking::onConstructVehicle>(*this);
    }

    using ActionList = edyn::action_list<VehicleAction>;

    void setSteering(float steering) {
        auto &input = m_registry->get<VehicleInput>(m_vehicle_entity);
        if (input.steering != steering) {
            input.steering = steering;
            m_registry->get<ActionList>(m_vehicle_entity)
                .actions.push_back(VehicleAction{VehicleSteeringAction{steering}});
            m_registry->get_or_emplace<edyn::dirty>(m_vehicle_entity).updated<ActionList>();
        }
    }

    void setThrottle(float throttle) {
        auto &input = m_registry->get<VehicleInput>(m_vehicle_entity);
        if (input.throttle != throttle) {
            input.throttle = throttle;
            m_registry->get<ActionList>(m_vehicle_entity)
                .actions.push_back(VehicleAction{VehicleThrottleAction{throttle}});
            m_registry->get_or_emplace<edyn::dirty>(m_vehicle_entity).updated<ActionList>();
        }
    }

    void setBrakes(float brakes) {
        auto &input = m_registry->get<VehicleInput>(m_vehicle_entity);
        if (input.brakes != brakes) {
            input.brakes = brakes;
            m_registry->get<ActionList>(m_vehicle_entity)
                .actions.push_back(VehicleAction{VehicleBrakeAction{brakes}});
            m_registry->get_or_emplace<edyn::dirty>(m_vehicle_entity).updated<ActionList>();
        }
    }

    void updatePhysics(float deltaTime) override {
        ExampleBasicNetworking::updatePhysics(deltaTime);

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
    }

    entt::entity m_vehicle_entity {entt::null};
};

ENTRY_IMPLEMENT_MAIN(
    ExampleVehicleNetworking
    , "00-vehicle-networking"
    , "Networked vehicle."
    , "https://github.com/xissburg/edyn"
    );
