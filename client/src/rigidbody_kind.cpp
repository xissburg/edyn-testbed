#include "edyn_example.hpp"
#include <edyn/comp/tag.hpp>
#include <edyn/config/solver_iteration_config.hpp>
#include <edyn/dynamics/moment_of_inertia.hpp>
#include <edyn/util/rigidbody.hpp>

class ExampleRigidBodyKind : public EdynExample
{
public:
    ExampleRigidBodyKind(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleRigidBodyKind() {}

    void createScene() override
    {
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        auto def = edyn::rigidbody_def();
        def.mass = 100;
        def.material->friction = 0.8;
        def.material->spin_friction = 0.0005;
        def.material->roll_friction = 0.0002;
        def.shape = edyn::sphere_shape{0.2};
        def.material->restitution = 1;
        def.position = {0, 3.4, 0};
        m_ball_entity[0] = edyn::make_rigidbody(*m_registry, def);

        def.linvel = {0, 2, 0};
        def.position = {0, 3.8, 0};
        m_ball_entity[1] = edyn::make_rigidbody(*m_registry, def);

        def.linvel = {0, 1.2, 0};
        def.position = {0, 4.5, 0};
        m_ball_entity[2] = edyn::make_rigidbody(*m_registry, def);
    }

    void updatePhysics(float deltaTime) override
    {
        m_timer += deltaTime;

        if (m_timer > 3) {
            m_timer = 0;
            auto kind = m_registry->all_of<edyn::dynamic_tag>(m_ball_entity[0]) ? edyn::rigidbody_kind::rb_static : edyn::rigidbody_kind::rb_dynamic;
            edyn::rigidbody_set_kind(*m_registry, m_ball_entity[0], kind);
        }

        EdynExample::updatePhysics(deltaTime);
    }

    float m_timer {};
    entt::entity m_ball_entity[3];
};

ENTRY_IMPLEMENT_MAIN(
    ExampleRigidBodyKind
    , "30-rigidbody-kind"
    , "Change rigid body kind."
    , "https://github.com/xissburg/edyn-testbed"
    );
