#include "edyn_example.hpp"
#include <edyn/comp/position.hpp>
#include <edyn/edyn.hpp>
#include <edyn/networking/client_networking_context.hpp>
#include <edyn/networking/networking.hpp>
#include <edyn/networking/packet/entity_request.hpp>
#include <edyn/networking/packet/packet.hpp>
#include <edyn/networking/packet/transient_snapshot.hpp>
#include <edyn/networking/remote_client.hpp>
#include <edyn/networking/server_side.hpp>
#include <edyn/networking/client_side.hpp>
#include <edyn/util/vector.hpp>

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

        std::vector<edyn::rigidbody_def> defs;

        for (int i = 0; i < 1; ++i) {
            for (int j = 0; j < 1; ++j) {
                for (int k = 0; k < 1; ++k) {
                    def.position = {edyn::scalar(0.4 * j),
                                    edyn::scalar(0.4 * i + 1.6),
                                    edyn::scalar(0.4 * k)};
                    defs.push_back(def);
                }
            }
        }

        edyn::batch_rigidbodies(*m_server_registry, defs);

        m_client_entity = edyn::server_make_client(*m_server_registry);
        auto &client = m_server_registry->get<edyn::remote_client>(m_client_entity);
        client.packet_sink().connect<&edyn::client_process_packet>(*m_registry);

        edyn::init_networking_client(*m_registry);
        auto &client_ctx = m_registry->ctx<edyn::client_networking_context>();
        client_ctx.request_entity_sink().connect<&ExampleBoxes::addToEntityRequest>(*this);
        client_ctx.packet_sink().connect<&ExampleBoxes::serverProcessPacket>(*this);
	}

    void serverProcessPacket(const edyn::edyn_packet &packet) {
        edyn::server_process_packet(*m_server_registry, m_client_entity, packet);
    }

    void destroyScene() override {
        m_server_registry.reset();
    }

    void updatePhysics(float deltaTime) override {

        edyn::update(*m_server_registry);

        if (m_counter++ % 10 == 0) {
            auto snapshot = edyn::server_get_transient_snapshot(*m_server_registry);
            edyn::client_process_packet(*m_registry, edyn::edyn_packet{std::move(snapshot)});
        }

        if (!m_entities_to_be_requested.empty()) {
            edyn::entity_request req;

            for (auto entity : m_entities_to_be_requested) {
                if (!edyn::vector_contains(m_entities_already_requested, entity)) {
                    req.entities.push_back(entity);
                }
            }

            edyn::server_process_packet(*m_server_registry, m_client_entity, edyn::edyn_packet{std::move(req)});

            m_entities_already_requested.insert(m_entities_already_requested.end(),
                                                m_entities_to_be_requested.begin(),
                                                m_entities_to_be_requested.end());
            m_entities_to_be_requested.clear();
        }

        edyn::update_networking_client(*m_registry);

        if (m_pick_entity != entt::null) {
            auto &client_ctx = m_registry->ctx<edyn::client_networking_context>();

            if (client_ctx.entity_map.has_loc(m_pick_entity)) {
                //auto remote_entity = client_ctx.entity_map.locrem(m_pick_entity);
                auto snapshot = edyn::transient_snapshot{};

                auto pool = std::make_unique<edyn::pool_snapshot<edyn::position>>();
                pool->component_index = edyn::tuple_index_of<edyn::position, unsigned>(edyn::networked_components);
                pool->pairs.emplace_back(m_pick_entity, m_registry->get<edyn::position>(m_pick_entity));
                snapshot.pools.push_back(std::move(pool));

                edyn::server_process_packet(*m_server_registry, m_client_entity, edyn::edyn_packet{std::move(snapshot)});
            }
        }

        EdynExample::updatePhysics(deltaTime);
    }

    void addToEntityRequest(entt::entity remoteEntity) {
        m_entities_to_be_requested.push_back(remoteEntity);
    }

    std::unique_ptr<entt::registry> m_server_registry;
    std::vector<entt::entity> m_entities_to_be_requested;
    std::vector<entt::entity> m_entities_already_requested;
    entt::entity m_client_entity;
    size_t m_counter{0};
};

ENTRY_IMPLEMENT_MAIN(
	ExampleBoxes
	,"01-boxes"
	, "Box stacking."
    , "https://github.com/xissburg/edyn-testbed"
    );
