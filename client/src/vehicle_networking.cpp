#include "networking.hpp"
#include "server_ports.hpp"
#include "vehicle_system.hpp"
#include "pick_input.hpp"
#include <edyn/networking/sys/client_side.hpp>
#include <edyn/util/island_util.hpp>

void PreStepUpdate(entt::registry &registry) {
    UpdatePickInput(registry);
    UpdateVehicles(registry);
}

class ExampleVehicleNetworking;

void OnEntityEnteredVehicle(ExampleVehicleNetworking &, entt::entity asset_entity);

class ExampleVehicleNetworking : public ExampleNetworking
{
public:
    ExampleVehicleNetworking(const char* _name, const char* _description, const char* _url)
        : ExampleNetworking(_name, _description, _url)
    {
        m_server_port = VehicleServerPort;
    }

    void createScene() override
    {
        m_vehicle_entity = entt::null;
        auto &registry = *m_registry;

        ExampleNetworking::createScene();

        RegisterNetworkedVehicleComponents(registry);
        RegisterVehicleComponents(registry);

        edyn::network_client_entity_entered_sink(registry).connect<&OnEntityEnteredVehicle>(*this);

        edyn::set_pre_step_callback(registry, &PreStepUpdate);
    }

    using ActionList = edyn::action_list<VehicleAction>;

    void insertAction(VehicleAction action) {
        if (!m_registry->all_of<ActionList>(m_vehicle_entity)) {
            m_registry->emplace<ActionList>(m_vehicle_entity);
        }

        m_registry->patch<ActionList>(m_vehicle_entity, [&](ActionList &list) {
            list.actions.push_back(action);
        });

        auto residents = std::vector<entt::entity>{};
        residents.push_back(m_vehicle_entity);
        edyn::wake_up_island_residents(*m_registry, residents);
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

void OnEntityEnteredVehicle(ExampleVehicleNetworking &example, entt::entity asset_entity) {
    auto &registry = *example.m_registry;
    auto &asset = registry.get<edyn::asset_ref>(asset_entity);

    if (asset.id == VehicleAssetID) {
        auto vehicleEntity = CreateVehicle(registry);
        auto emap = CreateVehicleAssetEntityMap(registry, vehicleEntity);
        edyn::client_link_asset(registry, asset_entity, emap);

        if (edyn::client_owns_entity(registry, asset_entity)) {
            example.m_vehicle_entity = vehicleEntity;
        }
    }
}

ENTRY_IMPLEMENT_MAIN(
    ExampleVehicleNetworking
    , "27-vehicle-networking"
    , "Networked vehicle."
    , "https://github.com/xissburg/edyn"
    );
