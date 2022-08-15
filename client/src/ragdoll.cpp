#include "edyn_example.hpp"
#include <edyn/util/ragdoll.hpp>

class ExampleRagDoll : public EdynExample
{
public:
    ExampleRagDoll(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    void createScene() override
    {
        m_fixed_dt_ms = 8;
        m_proportional_pick_stiffness = false;

        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        auto rag_def = edyn::ragdoll_simple_def{};
        rag_def.restitution = 0.3;
        rag_def.friction = 0.4;
        rag_def.position = {0, 1, 0};
        rag_def.shape_type = edyn::ragdoll_shape_type::capsule;
        edyn::make_ragdoll(*m_registry, rag_def);

        rag_def.height = 1;
        rag_def.weight = 40;
        rag_def.position = {0, 2.5, 0};
        rag_def.orientation = edyn::quaternion_axis_angle({0, 0, 1}, edyn::to_radians(24));
        edyn::make_ragdoll(*m_registry, rag_def);

        setPaused(true);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleRagDoll
    , "21-ragdoll"
    , "Rag doll."
    , "https://github.com/xissburg/edyn"
    );
