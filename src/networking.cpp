#include "edyn_example.hpp"
#include <edyn/comp/position.hpp>
#include <edyn/comp/present_orientation.hpp>
#include <edyn/comp/present_position.hpp>
#include <edyn/edyn.hpp>
#include <edyn/networking/client_networking_context.hpp>
#include <edyn/networking/networking.hpp>
#include <edyn/networking/packet/entity_request.hpp>
#include <edyn/networking/packet/edyn_packet.hpp>
#include <edyn/networking/packet/transient_snapshot.hpp>
#include <edyn/networking/packet/util/pool_snapshot.hpp>
#include <edyn/networking/remote_client.hpp>
#include <edyn/networking/server_side.hpp>
#include <edyn/networking/client_side.hpp>
#include <edyn/util/rigidbody.hpp>
#include <unordered_set>
#include <enet/enet.h>
#include <iostream>

class ExampleNetworking : public EdynExample
{
public:
	ExampleNetworking(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

    virtual ~ExampleNetworking() {}

    bool initEnet()
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

    bool connectToServer(const std::string &hostName, unsigned short port)
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

    void sendToServer(const uint8_t *data, size_t dataLength, uint32_t flags)
    {
        ENetPacket *packet = enet_packet_create(data, dataLength, flags);
        enet_peer_send(m_peer, (flags & ENET_PACKET_FLAG_RELIABLE) ? 0 : 1, packet);
    }

    void sendEdynPacketToServer(const edyn::packet::edyn_packet &packet)
    {
        uint32_t flags = 0;

        if (!std::holds_alternative<edyn::packet::transient_snapshot>(packet.var)) {
            flags |= ENET_PACKET_FLAG_RELIABLE;
        }

        auto data = std::vector<uint8_t>{};
        auto archive = edyn::memory_output_archive(data);
        archive(const_cast<edyn::packet::edyn_packet &>(packet));

        sendToServer(data.data(), data.size(), flags);
    }

    void onConstructRigidBody(entt::registry &registry, entt::entity entity) {
        if (!registry.any_of<edyn::present_position>(entity)) {
            registry.emplace<edyn::present_position>(entity);
            registry.emplace<edyn::present_orientation>(entity);
        }
    }

	void createScene() override
    {
        if (!initEnet()) {
            return;
        }

        edyn::init_networking_client(*m_registry);

        if (!connectToServer("localhost", 1337)) {
            return;
        }

        m_footer_text = "Connecting to server...";

        m_registry->on_construct<edyn::rigidbody_tag>().connect<&ExampleNetworking::onConstructRigidBody>(*this);
	}

    void destroyScene() override
    {
        EdynExample::destroyScene();

        if (m_peer) {
            enet_peer_disconnect(m_peer, 0);
        }

        enet_host_destroy(m_host);
        enet_deinitialize();

        edyn::deinit_networking_client(*m_registry);

        m_footer_text = m_default_footer_text;

        m_registry->on_construct<edyn::rigidbody_tag>().disconnect<&ExampleNetworking::onConstructRigidBody>(*this);
    }

    void updateNetworking()
    {
        ENetEvent event;

        while (enet_host_service(m_host, &event, 0) > 0) {
            switch (event.type) {
                case ENET_EVENT_TYPE_CONNECT: {
                    auto &edynCtx = m_registry->ctx<edyn::client_networking_context>();
                    edynCtx.packet_sink().connect<&ExampleNetworking::sendEdynPacketToServer>(*this);
                    m_footer_text = "Connected to server.";
                    break;
                }

                case ENET_EVENT_TYPE_DISCONNECT: {
                    auto &edynCtx = m_registry->ctx<edyn::client_networking_context>();
                    edynCtx.packet_sink().disconnect<&ExampleNetworking::sendEdynPacketToServer>(*this);
                    m_footer_text = "Disconnected.";
                    break;
                }

                case ENET_EVENT_TYPE_RECEIVE: {
                    auto archive = edyn::memory_input_archive(event.packet->data, event.packet->dataLength);
                    edyn::packet::edyn_packet packet;
                    archive(packet);

                    edyn::client_process_packet(*m_registry, packet);

                    /* Clean up the packet now that we're done using it. */
                    enet_packet_destroy(event.packet);

                    break;
                }

                default:
                    break;
            }
        }
    }

    void updatePhysics(float deltaTime) override
    {
        updateNetworking();
        EdynExample::updatePhysics(deltaTime);
        edyn::update_networking_client(*m_registry);

        if (m_pick_entity != entt::null) {
            if (!m_registry->any_of<edyn::networked_tag>(m_pick_entity)) {
                m_registry->emplace<edyn::networked_tag>(m_pick_entity);
                m_registry->emplace<edyn::networked_tag>(m_pick_constraint_entity);
            }

            auto snapshot = edyn::packet::transient_snapshot{};
            edyn::insert_entity_component<edyn::position>(*m_registry, m_pick_entity, snapshot.pools);
            auto packet = edyn::packet::edyn_packet{std::move(snapshot)};
            sendEdynPacketToServer(packet);
        }
    }

private:
    ENetHost *m_host {nullptr};
    ENetPeer *m_peer {nullptr};
};

ENTRY_IMPLEMENT_MAIN(
	ExampleNetworking
	, "00-networking"
	, "Networking."
    , "https://github.com/xissburg/edyn-testbed"
    );
