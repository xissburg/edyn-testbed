#include "basic_networking.hpp"
#include "server_ports.hpp"
#include <edyn/comp/continuous.hpp>
#include <edyn/comp/position.hpp>
#include <edyn/edyn.hpp>
#include <edyn/networking/networking.hpp>
#include <edyn/networking/networking_external.hpp>
#include <edyn/networking/packet/edyn_packet.hpp>
#include <edyn/util/rigidbody.hpp>
#include <unordered_set>
#include <enet/enet.h>
#include <iostream>
#include "pick_input.hpp"

void RegisterNetworkedComponents(entt::registry &);
void UnregisterNetworkedComponents(entt::registry &);
void cmdToggleExtrapolation(const void* _userData);

void PrintExtrapolationTimeoutWarning() {
    std::cout << "WARNING: Extrapolation timed out." << std::endl;
}

ExampleBasicNetworking::ExampleBasicNetworking(const char* _name, const char* _description, const char* _url)
    : EdynExample(_name, _description, _url)
    , m_server_port(NetworkingServerPort)
{

}

bool ExampleBasicNetworking::initEnet()
{
    if (enet_initialize () != 0) {
        std::cout << "An error occurred while initializing ENet." << std::endl;
        return false;
    }

    m_host = enet_host_create(NULL /* create a client host */,
                              2 /* allow 2 outgoing connections (one for game data stream) */,
                              2 /* allow up 2 channels to be used, 0 and 1 */,
                              0 /* 56K modem with 56 Kbps downstream bandwidth */,
                              0 /* 56K modem with 14 Kbps upstream bandwidth */);
    if (m_host == nullptr) {
        std::cout << "An error occurred while trying to create an ENet client host." << std::endl;
        return false;
    }

    return true;
}

bool ExampleBasicNetworking::connectToServer(const std::string &hostName, unsigned short port)
{
    ENetAddress address;
    enet_address_set_host(&address, hostName.c_str());
    address.port = port;

    /* Initiate the connection, allocating the two channels 0 and 1. */
    m_peer = enet_host_connect(m_host, &address, 2, 0);

    if (m_peer == nullptr) {
        std::cout << "No available peers for initiating an ENet connection." << std::endl;
        return false;
    }

    return true;
}

void ExampleBasicNetworking::sendToServer(const uint8_t *data, size_t dataLength, uint32_t flags)
{
    ENetPacket *packet = enet_packet_create(data, dataLength, flags);
    enet_peer_send(m_peer, (flags & ENET_PACKET_FLAG_RELIABLE) ? 0 : 1, packet);
}

void ExampleBasicNetworking::sendEdynPacketToServer(const edyn::packet::edyn_packet &packet)
{
    uint32_t flags = 0;

    if (edyn::should_send_reliably(packet)) {
        flags |= ENET_PACKET_FLAG_RELIABLE;
    }

    auto data = std::vector<uint8_t>{};
    auto archive = edyn::memory_output_archive(data);
    archive(const_cast<edyn::packet::edyn_packet &>(packet));

    sendToServer(data.data(), data.size(), flags);
}

void ExampleBasicNetworking::onConstructRigidBody(entt::registry &registry, entt::entity entity)
{
    if (registry.all_of<edyn::dynamic_tag>(entity) && !registry.any_of<edyn::present_position>(entity)) {
        registry.emplace<edyn::present_position>(entity);
        registry.emplace<edyn::present_orientation>(entity);
    }
}

void ExampleBasicNetworking::toggleExtrapolation()
{
    edyn::toggle_network_client_extrapolation_enabled(*m_registry);
}

void ExampleBasicNetworking::createScene()
{
    if (!initEnet()) {
        return;
    }

    edyn::init_network_client(*m_registry);

    RegisterNetworkedComponents(*m_registry);
    edyn::set_external_system_pre_step(*m_registry, &UpdatePickInput);

    edyn::network_client_extrapolation_timeout_sink(*m_registry).connect<&PrintExtrapolationTimeoutWarning>();

    if (!connectToServer("localhost", m_server_port)) {
        return;
    }

    m_footer_text = "Connecting to server...";

    m_registry->on_construct<edyn::rigidbody_tag>().connect<&ExampleBasicNetworking::onConstructRigidBody>(*this);

    // Input bindings
    m_network_bindings = (InputBinding*)BX_ALLOC(entry::getAllocator(), sizeof(InputBinding)*2);
    m_network_bindings[0].set(entry::Key::KeyM, entry::Modifier::None, 1, cmdToggleExtrapolation,  this);
    m_network_bindings[1].end();

    inputAddBindings("networking", m_network_bindings);
}

void ExampleBasicNetworking::destroyScene()
{
    EdynExample::destroyScene();

    if (m_peer) {
        enet_peer_disconnect(m_peer, 0);
    }

    enet_host_destroy(m_host);
    enet_deinitialize();

    edyn::deinit_network_client(*m_registry);

    edyn::remove_external_systems(*m_registry);
    UnregisterNetworkedComponents(*m_registry);

    m_footer_text = m_default_footer_text;

    m_registry->on_construct<edyn::rigidbody_tag>().disconnect<&ExampleBasicNetworking::onConstructRigidBody>(*this);

    inputRemoveBindings("networking");
    BX_FREE(entry::getAllocator(), m_network_bindings);
}

void ExampleBasicNetworking::updateNetworking()
{
    ENetEvent event;

    while (enet_host_service(m_host, &event, 0) > 0) {
        switch (event.type) {
            case ENET_EVENT_TYPE_CONNECT: {
                edyn::set_network_client_max_concurrent_extrapolations(*m_registry, 1);
                edyn::network_client_packet_sink(*m_registry)
                    .connect<&ExampleBasicNetworking::sendEdynPacketToServer>(*this);
                m_footer_text = "Connected to server.";
                break;
            }

            case ENET_EVENT_TYPE_DISCONNECT: {
                edyn::network_client_packet_sink(*m_registry)
                    .disconnect<&ExampleBasicNetworking::sendEdynPacketToServer>(*this);
                m_footer_text = "Disconnected.";
                break;
            }

            case ENET_EVENT_TYPE_RECEIVE: {
                auto archive = edyn::memory_input_archive(event.packet->data, event.packet->dataLength);
                edyn::packet::edyn_packet packet;
                archive(packet);

                if (!archive.failed()) {
                    edyn::client_receive_packet(*m_registry, packet);

                    // Assign server settings to UI.
                    if (std::holds_alternative<edyn::packet::server_settings>(packet.var)) {
                        auto &settings = std::get<edyn::packet::server_settings>(packet.var);
                        m_fixed_dt_ms = settings.fixed_dt * 1000;
                        m_num_velocity_iterations = settings.num_solver_velocity_iterations;
                        m_num_position_iterations = settings.num_solver_position_iterations;
                    }
                }

                /* Clean up the packet now that we're done using it. */
                enet_packet_destroy(event.packet);

                break;
            }

            default:
                break;
        }
    }

    if (m_peer) {
        edyn::set_network_client_round_trip_time(*m_registry, 1e-3 * m_peer->roundTripTime);
    }
}

void ExampleBasicNetworking::updatePhysics(float deltaTime)
{
    if (m_pick_entity != entt::null) {
        if (!m_registry->any_of<edyn::networked_tag>(m_pick_entity)) {
            // Make pick entity networked.
            m_registry->emplace<edyn::networked_tag>(m_pick_entity);
            m_registry->emplace<edyn::networked_tag>(m_pick_constraint_entity);
            m_registry->emplace<PickInput>(m_pick_entity);
            m_registry->emplace<edyn::continuous>(m_pick_entity).insert(edyn::get_component_index<edyn::position>(*m_registry));
            // Marking it as dirty will not cause a general_snapshot to be sent
            // because PickInput is a transient component.
            m_registry->get_or_emplace<edyn::dirty>(m_pick_entity).created<PickInput>();
        }

        m_registry->get<PickInput>(m_pick_entity).position = m_registry->get<edyn::position>(m_pick_entity);
        m_registry->get_or_emplace<edyn::dirty>(m_pick_entity).updated<PickInput>();
    }

    updateNetworking();
    edyn::update_network_client(*m_registry);
    EdynExample::updatePhysics(deltaTime);
    enet_host_flush(m_host);
}

void cmdToggleExtrapolation(const void* _userData) {
    ((ExampleBasicNetworking *)_userData)->toggleExtrapolation();
}

ENTRY_IMPLEMENT_MAIN(
    ExampleBasicNetworking
    , "25-networking"
    , "Basic networking."
    , "https://github.com/xissburg/edyn-testbed"
    );
