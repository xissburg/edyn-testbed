#include "edyn_example.hpp"

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
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.mass = 10;
        def.material->friction = 0.8;
        def.material->restitution = 0;
        def.shape = edyn::box_shape{0.2, 0.2, 0.2};
        def.update_inertia();
        def.continuous_contacts = true;

        std::vector<edyn::rigidbody_def> defs;

        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                for (int k = 0; k < 5; ++k) {
                    def.position = {edyn::scalar(0.4 * j),
                                    edyn::scalar(0.4 * i + 0.6),
                                    edyn::scalar(0.4 * k)};
                    defs.push_back(def);
                }
            }
        }

        edyn::batch_rigidbodies(*m_registry, defs);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleBoxes
    , "01-boxes"
    , "Box stacking."
    , "https://github.com/xissburg/edyn-testbed"
    );
