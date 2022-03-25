#ifndef EDYN_TESTBED_BASIC_NETWORKING_EXAMPLE_HPP
#define EDYN_TESTBED_BASIC_NETWORKING_EXAMPLE_HPP

#include "edyn_example.hpp"
#include <cstdint>
#include <edyn/networking/networking.hpp>
#include <enet/enet.h>

class ExampleBasicNetworking : public EdynExample
{
public:
    ExampleBasicNetworking(const char* _name, const char* _description, const char* _url);

    virtual ~ExampleBasicNetworking() {}

    bool initEnet();

    bool connectToServer(const std::string &hostName, unsigned short port);

    void sendToServer(const uint8_t *data, size_t dataLength, uint32_t flags);

    void sendEdynPacketToServer(const edyn::packet::edyn_packet &packet);

    void onConstructRigidBody(entt::registry &registry, entt::entity entity);

    void toggleExtrapolation();

    void createScene() override;

    void destroyScene() override;

    void updateNetworking();

    void updatePhysics(float deltaTime) override;

private:
    ENetHost *m_host {nullptr};
    ENetPeer *m_peer {nullptr};
    InputBinding* m_network_bindings;

protected:
    uint16_t m_server_port;
};

#endif // EDYN_TESTBED_BASIC_NETWORKING_EXAMPLE_HPP
