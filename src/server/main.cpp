#include <edyn/networking/packet/set_playout_delay.hpp>
#include <enet/enet.h>
#include <entt/entity/entity.hpp>
#include <entt/entity/registry.hpp>
#include <edyn/edyn.hpp>
#include <edyn/networking/networking.hpp>
#include <iostream>

struct PeerID {
    unsigned short value;
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

ENetHost * init_enet() {
    // Init ENet.
    if (enet_initialize () != 0) {
        std::cout << "An error occurred while initializing ENet." << std::endl;
        return nullptr;
    }

    ENetAddress address;
    address.host = ENET_HOST_ANY;
    enet_address_set_host(&address, "0.0.0.0");
    address.port = 1337;

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

template<typename ClientEntityMap>
void update_enet(entt::registry &registry, ClientEntityMap &clientEntityMap) {
    auto *host = registry.ctx<ENetHost *>();
    ENetEvent event;

    while (enet_host_service(host, &event, 0) > 0) {
        const auto peerID = event.peer->incomingPeerID;

        switch (event.type) {
            case ENET_EVENT_TYPE_CONNECT: {
                auto clientEntity = edyn::server_make_client(registry);
                registry.emplace<PeerID>(clientEntity, peerID);
                clientEntityMap[peerID] = clientEntity;

                auto &client = registry.get<edyn::remote_client>(clientEntity);
                client.snapshot_rate = 1;
                client.playout_delay = 0.3;
                client.packet_sink().connect<&send_edyn_packet_to_client>(registry);

                auto delay = edyn::packet::set_playout_delay{client.playout_delay};
                send_edyn_packet_to_client(registry, clientEntity, edyn::packet::edyn_packet{delay});

                std::cout << "Connected " << std::hex << entt::to_integral(clientEntity) << std::endl;
                break;
            }

            case ENET_EVENT_TYPE_DISCONNECT: {
                auto clientEntity = clientEntityMap.at(peerID);
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
                    edyn::server_handle_packet(registry, clientEntity, packet);
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

void create_scene(entt::registry &registry) {
    // Create floor.
    auto floor_def = edyn::rigidbody_def();
    floor_def.kind = edyn::rigidbody_kind::rb_static;
    floor_def.material->restitution = 0;
    floor_def.material->friction = 0.5;
    floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
    edyn::make_rigidbody(registry, floor_def);

    // Create boxes.
    auto def = edyn::rigidbody_def();
    def.mass = 10;
    def.material->friction = 0.8;
    def.material->restitution = 0;
    def.shape = edyn::box_shape{0.2, 0.2, 0.2};
    def.update_inertia();
    def.continuous_contacts = true;
    def.sleeping_disabled = true;
    def.networked = true;

    std::vector<edyn::rigidbody_def> defs;

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                def.position = {edyn::scalar(0.4 * j),
                                edyn::scalar(0.4 * i + 0.6),
                                edyn::scalar(0.4 * k)};
                defs.push_back(def);
            }
        }
    }

    edyn::batch_rigidbodies(registry, defs);
}

int main() {
    entt::registry registry;

    // Init Edyn.
    edyn::init();
    edyn::attach(registry);

    // Init networking.
    auto *host = init_enet();

    if (host == nullptr) {
        return -1;
    }

    registry.set<ENetHost *>(host);

    edyn::init_networking_server(registry);

    std::unordered_map<unsigned short, entt::entity> clientEntityMap;

    create_scene(registry);

    // Use a PID to keep updates at a fixed and controlled rate.
    auto updateRate = 120;
    auto desiredDt = 1.0 / updateRate;
    auto proportionalTerm = 0.18;
    auto integralTerm = 0.06;
    auto iTerm = 0.0;
    auto time = edyn::performance_time();

    while (true) {
        update_enet(registry, clientEntityMap);
        edyn::update_networking_server(registry);
        edyn::update(registry);

        // Apply delay to maintain a fixed update rate.
        auto t1 = edyn::performance_time();
        auto dt = t1 - time;
        time = t1;

        auto error = desiredDt - dt;
        iTerm = std::max(-1.0, std::min(iTerm + integralTerm * error, 1.0));
        auto delay = std::max(0.0, proportionalTerm * error + iTerm);
        edyn::delay(delay * 1000);
    }

    edyn::detach(registry);
    edyn::deinit_networking_server(registry);
    edyn::deinit();

    enet_host_destroy(host);
    enet_deinitialize();

    return 0;
}
