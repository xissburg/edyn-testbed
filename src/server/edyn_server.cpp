#include "edyn_server.hpp"
#include <edyn/networking/networking.hpp>
#include <edyn/networking/sys/server_side.hpp>
#include <entt/entity/registry.hpp>
#include <edyn/edyn.hpp>
#include <iostream>

struct PeerID {
    unsigned short value;
};

struct ClientEntityMap {
    std::unordered_map<unsigned short, entt::entity> map;
};

void send_to_client(ENetHost *host, const uint8_t *data, size_t dataLength, unsigned short peerID, uint32_t flags)
{
    assert(host && peerID < host->peerCount);
    ENetPacket *packet = enet_packet_create(data, dataLength, flags);
    ENetPeer *peer = &host->peers[peerID];
    enet_peer_send(peer, (flags & ENET_PACKET_FLAG_RELIABLE) ? 0 : 1, packet);
}

void send_edyn_packet_to_client(entt::registry &registry, entt::entity clientEntity, const edyn::packet::edyn_packet &packet)
{
    uint32_t flags = 0;

    if (!std::holds_alternative<edyn::packet::transient_snapshot>(packet.var)) {
        flags |= ENET_PACKET_FLAG_RELIABLE;
    }

    auto data = std::vector<uint8_t>{};
    auto archive = edyn::memory_output_archive(data);
    archive(const_cast<edyn::packet::edyn_packet &>(packet));

    auto peerID = registry.get<PeerID>(clientEntity).value;
    auto *host = registry.ctx<ENetHost *>();
    send_to_client(host, data.data(), data.size(), peerID, flags);
}

ENetHost * init_enet(uint16_t port) {
    // Init ENet.
    if (enet_initialize () != 0) {
        std::cout << "An error occurred while initializing ENet." << std::endl;
        return nullptr;
    }

    ENetAddress address;
    address.host = ENET_HOST_ANY;
    enet_address_set_host(&address, "0.0.0.0");
    address.port = port;

    auto *host = enet_host_create(&address /* the address to bind the server host to */,
                                   32      /* allow up to 32 clients and/or outgoing connections */,
                                    2      /* allow up to 2 channels to be used, 0 and 1 */,
                                    0      /* assume any amount of incoming bandwidth */,
                                    0      /* assume any amount of outgoing bandwidth */);
    if (host == nullptr) {
        std::cout << "An error occurred while trying to create an ENet server host." << std::endl;
        return nullptr;
    }

    return host;
}

bool edyn_server_init(entt::registry &registry, uint16_t port) {
    // Init Edyn.
    edyn::init();
    edyn::attach(registry);

    // Init networking.
    auto *host = init_enet(port);

    if (host == nullptr) {
        return false;
    }

    registry.set<ENetHost *>(host);
    registry.set<ClientEntityMap>();

    edyn::init_network_server(registry);

    return true;
}

void edyn_server_deinit(entt::registry &registry) {
    edyn::detach(registry);
    edyn::deinit_network_server(registry);
    edyn::deinit();

    auto *host = registry.ctx<ENetHost *>();
    enet_host_destroy(host);
    enet_deinitialize();

    registry.unset<ENetHost *>();
    registry.unset<ClientEntityMap>();
}

void edyn_server_process_packets(entt::registry &registry) {
    auto *host = registry.ctx<ENetHost *>();
    auto &clientEntityMap = registry.ctx<ClientEntityMap>().map;
    ENetEvent event;

    while (enet_host_service(host, &event, 0) > 0) {
        const auto peerID = event.peer->incomingPeerID;

        switch (event.type) {
            case ENET_EVENT_TYPE_CONNECT: {
                enet_peer_timeout(event.peer, 0, 10000, 30000);

                auto clientEntity = edyn::server_make_client(registry);
                registry.emplace<PeerID>(clientEntity, peerID);
                clientEntityMap[peerID] = clientEntity;

                auto &client = registry.get<edyn::remote_client>(clientEntity);
                client.snapshot_rate = 10;
                client.packet_sink().connect<&send_edyn_packet_to_client>(registry);

                auto delay = edyn::packet::set_playout_delay{client.playout_delay};
                send_edyn_packet_to_client(registry, clientEntity, edyn::packet::edyn_packet{delay});

                std::cout << "Connected " << std::hex << entt::to_integral(clientEntity) << std::endl;
                break;
            }

            case ENET_EVENT_TYPE_DISCONNECT: {
                auto clientEntity = clientEntityMap.at(peerID);
                auto &remoteClient = registry.get<edyn::remote_client>(clientEntity);

                for (auto entity : remoteClient.owned_entities) {
                    if (registry.valid(entity)) {
                        registry.destroy(entity);
                    }
                }

                registry.destroy(clientEntity);
                clientEntityMap.erase(peerID);
                std::cout << "Disconnected " << std::hex << entt::to_integral(clientEntity) << std::endl;
                break;
            }

            case ENET_EVENT_TYPE_RECEIVE: {
                auto archive = edyn::memory_input_archive(event.packet->data, event.packet->dataLength);
                edyn::packet::edyn_packet packet;
                archive(packet);

                if (clientEntityMap.count(peerID)) {
                    auto clientEntity = clientEntityMap.at(peerID);
                    edyn::server_receive_packet(registry, clientEntity, packet);
                }

                /* Clean up the packet now that we're done using it. */
                enet_packet_destroy(event.packet);

                break;
            }

            default:
                break;
        }
    }
}

void edyn_server_update_latencies(entt::registry &registry) {
    auto &clientEntityMap = registry.ctx<ClientEntityMap>().map;
    auto *host = registry.ctx<ENetHost *>();

    for (auto [peerID, clientEntity] : clientEntityMap) {
        auto *peer = &host->peers[peerID];
        edyn::server_set_client_round_trip_time(registry, clientEntity, peer->roundTripTime * 0.001);
    }
}

void edyn_server_run(entt::registry &registry) {
    // Use a PID to keep updates at a fixed and controlled rate.
    auto updateRate = 120;
    auto desiredDt = 1.0 / updateRate;
    auto proportionalTerm = 0.18;
    auto integralTerm = 0.06;
    auto iTerm = 0.0;
    auto time = edyn::performance_time();

    while (true) {
        edyn_server_process_packets(registry);
        edyn_server_update_latencies(registry);
        edyn::update_network_server(registry);
        edyn::update(registry);
        edyn_server_update(registry);

        // Apply delay to maintain a fixed update rate.
        auto t1 = edyn::performance_time();
        auto dt = t1 - time;
        time = t1;

        auto error = desiredDt - dt;
        iTerm = std::max(-1.0, std::min(iTerm + integralTerm * error, 1.0));
        auto delay = std::max(0.0, proportionalTerm * error + iTerm);
        edyn::delay(delay * 1000);
    }
}