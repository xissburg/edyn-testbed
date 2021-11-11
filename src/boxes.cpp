#include "edyn_example.hpp"
#include <edyn/edyn.hpp>
#include <edyn/networking/client_networking_context.hpp>
#include <edyn/networking/networking.hpp>
#include <edyn/networking/packet/entity_request.hpp>
#include <edyn/networking/packet/edyn_packet.hpp>
#include <edyn/networking/packet/transient_snapshot.hpp>
#include <edyn/networking/remote_client.hpp>
#include <edyn/networking/server_side.hpp>
#include <edyn/networking/client_side.hpp>
#include <edyn/util/rigidbody.hpp>
#include <unordered_set>

class ExampleBoxes;

struct LocalClientContext {
    ExampleBoxes *example;
    entt::entity client_entity;
};

void ServerEnqueuePacket(LocalClientContext &, const edyn::packet::edyn_packet &);

class ExampleBoxes : public EdynExample
{
public:
	ExampleBoxes(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

    virtual ~ExampleBoxes() {}

	void createScene() override
	{
        m_server_registry.reset(new entt::registry);
        edyn::attach(*m_server_registry);
        edyn::init_networking_server(*m_server_registry);

        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        floor_def.networked = true;
        edyn::make_rigidbody(*m_server_registry, floor_def);

        // Add some boxes.
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

        for (int i = 0; i < 1; ++i) {
            for (int j = 0; j < 2; ++j) {
                for (int k = 0; k < 1; ++k) {
                    def.position = {edyn::scalar(0.4 * j),
                                    edyn::scalar(0.4 * i + 0.6),
                                    edyn::scalar(0.4 * k)};
                    defs.push_back(def);
                }
            }
        }

        edyn::batch_rigidbodies(*m_server_registry, defs);

        // Create clients in server.
        m_client_entities[0] = edyn::server_make_client(*m_server_registry);
        m_client_entities[1] = edyn::server_make_client(*m_server_registry);

        {
            auto &client = m_server_registry->get<edyn::remote_client>(m_client_entities[0]);
            // Whenever a packet is generated for this client, add it to the client
            // packet queue with a timestamp in the future meaning that it will be
            // dispatched to the server later, thus simulating latency.
            client.packet_sink().connect<&ExampleBoxes::clientEnqueuePacket>(*this);
            edyn::server_set_client_latency(*m_server_registry, m_client_entities[0], m_round_trip_time / 2);
        }

        {
            auto &client = m_server_registry->get<edyn::remote_client>(m_client_entities[1]);
            client.packet_sink().connect<&ExampleBoxes::clientEnqueuePacket>(*this);
            edyn::server_set_client_latency(*m_server_registry, m_client_entities[1], m_round_trip_time / 2);
        }

        // Initialize clients.
        {
            m_client_ctxes[0].example = this;
            m_client_ctxes[0].client_entity = m_client_entities[0];

            edyn::init_networking_client(*m_registry);
            auto &client_ctx = m_registry->ctx<edyn::client_networking_context>();
            // Whenever a packet is generated in this client, add it to the queue
            // of server packets to be dispatched later, thus simulating latency.
            client_ctx.packet_sink().connect<&ServerEnqueuePacket>(m_client_ctxes[0]);
        }

        {
            m_client_ctxes[1].example = this;
            m_client_ctxes[1].client_entity = m_client_entities[1];

            m_client_registry.reset(new entt::registry);
            edyn::attach(*m_client_registry);
            edyn::init_networking_client(*m_client_registry);
            auto &client_ctx = m_client_registry->ctx<edyn::client_networking_context>();
            client_ctx.packet_sink().connect<&ServerEnqueuePacket>(m_client_ctxes[1]);

            def.position = {1.6, 1.2, 0};
            edyn::make_rigidbody(*m_client_registry, def);
        }
	}

    void destroyScene() override {
        EdynExample::destroyScene();
        m_server_registry.reset();
    }

    void serverEnqueuePacket(entt::entity client_entity, const edyn::packet::edyn_packet &packet) {
        auto p = packet;
        p.timestamp = edyn::performance_time() + m_round_trip_time / 2;
        m_server_packet_queue.emplace_back(client_entity, p);
    }

    void clientEnqueuePacket(entt::entity client_entity, const edyn::packet::edyn_packet &packet) {
        auto p = packet;
        p.timestamp = edyn::performance_time() + m_round_trip_time / 2;
        m_client_packet_queue[client_entity].push_back(p);
    }

    void serverProcessPackets() {
        // Process packets up until the current time.
        auto timestamp = edyn::performance_time();
        auto first = m_server_packet_queue.begin();
        auto last = std::find_if(first, m_server_packet_queue.end(), [&] (auto &&pair) {
            return pair.second.timestamp > timestamp;
        });

        for (auto it = first; it != last; ++it) {
            edyn::server_handle_packet(*m_server_registry, it->first, it->second);
        }

        m_server_packet_queue.erase(first, last);

        edyn::server_process_packets(*m_server_registry);
    }

    void clientProcessPackets(size_t client_index) {
        auto timestamp = edyn::performance_time();
        auto &queue = m_client_packet_queue[m_client_entities[client_index]];
        auto first = queue.begin();
        auto last = std::find_if(first, queue.end(), [&] (auto &&packet) {
            return packet.timestamp > timestamp;
        });

        auto &registry = client_index == 0 ? *m_registry : *m_client_registry;

        for (auto it = first; it != last; ++it) {
            edyn::client_process_packet(registry, *it);
        }

        queue.erase(first, last);
    }

    void updatePhysics(float deltaTime) override {
        serverProcessPackets();
        edyn::update(*m_server_registry);
        edyn::update_networking_server(*m_server_registry);

        clientProcessPackets(0);

        EdynExample::updatePhysics(deltaTime);
        edyn::update_networking_client(*m_registry);

        clientProcessPackets(1);
        edyn::update(*m_client_registry);
        edyn::update_networking_client(*m_client_registry);

        if (m_counter++ % 3 == 0) {
            for (auto client_entity : m_client_entities) {
                auto snapshot = edyn::server_get_transient_snapshot(*m_server_registry, client_entity);
                auto packet = edyn::packet::edyn_packet{std::move(snapshot)};
                clientEnqueuePacket(client_entity, packet);
            }
        }

        if (m_pick_entity != entt::null) {
            if (!m_registry->any_of<edyn::networked_tag>(m_pick_entity)) {
                m_registry->emplace<edyn::networked_tag>(m_pick_entity);
                m_registry->emplace<edyn::networked_tag>(m_pick_constraint_entity);
            }

            auto snapshot = edyn::packet::transient_snapshot{};
            snapshot.positions.emplace_back(m_pick_entity, m_registry->get<edyn::position>(m_pick_entity));
            auto packet = edyn::packet::edyn_packet{std::move(snapshot)};
            serverEnqueuePacket(m_client_entities[0], packet);
        }
    }

    std::unique_ptr<entt::registry> m_server_registry;
    std::unique_ptr<entt::registry> m_client_registry;
    std::array<entt::entity, 2> m_client_entities;
    std::array<LocalClientContext, 2> m_client_ctxes;
    size_t m_counter{0};
    std::vector<std::pair<entt::entity, edyn::packet::edyn_packet>> m_server_packet_queue;
    std::map<entt::entity, std::vector<edyn::packet::edyn_packet>> m_client_packet_queue;
    double m_round_trip_time {0.1};
};

void ServerEnqueuePacket(LocalClientContext &ctx, const edyn::packet::edyn_packet &packet) {
    ctx.example->serverEnqueuePacket(ctx.client_entity, packet);
}

ENTRY_IMPLEMENT_MAIN(
	ExampleBoxes
	, "01-boxes"
	, "Box stacking."
    , "https://github.com/xissburg/edyn-testbed"
    );
