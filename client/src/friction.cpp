#include "edyn_example.hpp"

class ExampleFriction : public EdynExample
{
public:
    ExampleFriction(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleFriction() {}

    void createScene() override
    {
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 0;
        floor_def.material->friction = 1;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.mass = 10;
        def.material->friction = 1;
        def.material->restitution = 0;
        def.shape = edyn::box_shape{0.1, 0.1, 0.1};
        def.update_inertia();
        def.continuous_contacts = true;

        size_t num_boxes = 24;
        float radius = 1.2;

        for (size_t i = 0; i < num_boxes; ++i) {
            auto angle = float(i) / float(num_boxes) * edyn::pi2;
            def.position = {edyn::scalar(std::cos(angle) * radius),
                            edyn::scalar(0.1),
                            edyn::scalar(std::sin(angle) * radius)};
            def.linvel = {edyn::scalar(std::cos(angle)), edyn::scalar(0), edyn::scalar(std::sin(angle))};
            def.linvel *= 5;
            def.material->friction = 1 - std::sqrt(float(i) / float(num_boxes));
            edyn::make_rigidbody(*m_registry, def);
        }
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleFriction
    ,"16-friction"
    , "Friction."
    , "https://github.com/xissburg/edyn-testbed"
    );
