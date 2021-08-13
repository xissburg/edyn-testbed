#include "edyn_example.hpp"
#include <edyn/comp/position.hpp>
#include <edyn/comp/tag.hpp>
#include <edyn/edyn.hpp>
#include <edyn/networking/client_networking_context.hpp>
#include <edyn/networking/networking.hpp>
#include <edyn/networking/packet/entity_request.hpp>
#include <edyn/networking/packet/edyn_packet.hpp>
#include <edyn/networking/packet/transient_snapshot.hpp>
#include <edyn/networking/remote_client.hpp>
#include <edyn/networking/server_side.hpp>
#include <edyn/networking/client_side.hpp>
#include <edyn/time/time.hpp>
#include <unordered_set>

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
        floor_def.material = {0, 0.5}; // {restitution, friction}
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
            for (int j = 0; j <2; ++j) {
                for (int k = 0; k < 1; ++k) {
                    def.position = {edyn::scalar(0.4 * j),
                                    edyn::scalar(0.4 * i + 0.6),
                                    edyn::scalar(0.4 * k)};
                    defs.push_back(def);
                }
            }
        }

        edyn::batch_rigidbodies(*m_server_registry, defs);

        m_client_entity = edyn::server_make_client(*m_server_registry);
        auto &client = m_server_registry->get<edyn::remote_client>(m_client_entity);
        client.packet_sink().connect<&edyn::client_process_packet>(*m_registry);
        edyn::server_set_client_latency(*m_server_registry, m_client_entity, 2);

        edyn::init_networking_client(*m_registry);
        auto &client_ctx = m_registry->ctx<edyn::client_networking_context>();
        client_ctx.packet_sink().connect<&ExampleBoxes::serverEnqueuePacket>(*this);
	}

    void serverEnqueuePacket(const edyn::packet::edyn_packet &packet) {
        auto p = packet;
        p.timestamp = edyn::performance_time() + 2;
        m_server_packet_queue.push_back(p);
    }

    void clientEnqueuePacket(const edyn::packet::edyn_packet &packet) {
        auto p = packet;
        p.timestamp = edyn::performance_time() + 2;
        m_client_packet_queue.push_back(p);
    }

    void destroyScene() override {
        m_server_registry.reset();
    }

    void serverProcessPackets() {
        auto timestamp = edyn::performance_time();
        auto first = m_server_packet_queue.begin();
        auto last = std::find_if(first, m_server_packet_queue.end(), [&] (auto &&packet) {
            return packet.timestamp > timestamp;
        });

        for (auto it = first; it != last; ++it) {
            edyn::server_process_packet(*m_server_registry, m_client_entity, *it);
        }

        m_server_packet_queue.erase(first, last);
    }

    void clientProcessPackets() {
        auto timestamp = edyn::performance_time();
        auto first = m_client_packet_queue.begin();
        auto last = std::find_if(first, m_client_packet_queue.end(), [&] (auto &&packet) {
            return packet.timestamp > timestamp;
        });

        for (auto it = first; it != last; ++it) {
            edyn::client_process_packet(*m_registry, *it);
        }

        m_client_packet_queue.erase(first, last);
    }

    void updatePhysics(float deltaTime) override {
        serverProcessPackets();
        edyn::update(*m_server_registry);
        edyn::update_networking_server(*m_server_registry);

        clientProcessPackets();
        edyn::update_networking_client(*m_registry);
        EdynExample::updatePhysics(deltaTime);

        if (m_counter++ % 5 == 0) {
            auto snapshot = edyn::server_get_transient_snapshot(*m_server_registry);
            auto packet = edyn::packet::edyn_packet{std::move(snapshot)};
            clientEnqueuePacket(packet);
        }

        if (m_pick_entity != entt::null) {
            if (!m_registry->has<edyn::networked_tag>(m_pick_entity)) {
                m_registry->emplace<edyn::networked_tag>(m_pick_entity);
                m_registry->emplace<edyn::networked_tag>(m_pick_constraint_entity);
            }

            auto &client_ctx = m_registry->ctx<edyn::client_networking_context>();

            if (client_ctx.entity_map.has_loc(m_pick_entity)) {
                auto snapshot = edyn::packet::transient_snapshot{};
                snapshot.positions.emplace_back(m_pick_entity, m_registry->get<edyn::position>(m_pick_entity));
                auto packet = edyn::packet::edyn_packet{std::move(snapshot)};
                serverEnqueuePacket(packet);
            }
        }
    }

    std::unique_ptr<entt::registry> m_server_registry;
    entt::entity m_client_entity;
    size_t m_counter{0};
    std::vector<edyn::packet::edyn_packet> m_server_packet_queue;
    std::vector<edyn::packet::edyn_packet> m_client_packet_queue;
};

ENTRY_IMPLEMENT_MAIN(
	ExampleBoxes
	,"01-boxes"
	, "Box stacking."
    , "https://github.com/xissburg/edyn-testbed"
    );
