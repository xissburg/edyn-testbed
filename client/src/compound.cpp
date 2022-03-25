#include "edyn_example.hpp"

class ExampleCompound : public EdynExample
{
public:
    ExampleCompound(const char* _name, const char* _description, const char* _url)
      : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleCompound() {}

    void createScene() override
    {
        // Create entities.
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 1;
        floor_def.material->friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Create a chain link using multiple shapes.
        // Increasing `max_contacts` in "edyn/config/constants.hpp" will yield
        // more stable results and lower likelihood of separation.
    #define COMPOUND_TYPE_MANUAL 0
    #define COMPOUND_TYPE_OBJ 1
    #define COMPOUND_TYPE COMPOUND_TYPE_OBJ

    #if COMPOUND_TYPE == COMPOUND_TYPE_MANUAL
        auto compound = edyn::compound_shape{};
        compound.add_shape(edyn::box_shape{0.05, 0.05, 0.25}, {-0.45, 0, 0}, edyn::quaternion_identity);
        compound.add_shape(edyn::box_shape{0.05, 0.05, 0.25}, { 0.0, 0, 0}, edyn::quaternion_identity);
        compound.add_shape(edyn::box_shape{0.05, 0.05, 0.25}, { 0.45, 0, 0}, edyn::quaternion_identity);
        compound.add_shape(edyn::box_shape{ 0.5, 0.05, 0.05}, { 0.0, 0, 0.2}, edyn::quaternion_identity);
        compound.add_shape(edyn::box_shape{ 0.5, 0.05, 0.05}, { 0.0, 0, -0.2}, edyn::quaternion_identity);
        compound.finish();
    #elif COMPOUND_TYPE == COMPOUND_TYPE_OBJ
        auto obj_path = "../../../edyn-testbed/resources/chain_link.obj";
        auto compound = edyn::compound_shape(obj_path, edyn::vector3_zero,
                                             edyn::quaternion_axis_angle({0, 1, 0}, edyn::pi * 0.5));
    #endif

        auto dyn_def = edyn::rigidbody_def();
        dyn_def.continuous_contacts = true;
        dyn_def.mass = 50;
        dyn_def.shape = compound;
        dyn_def.update_inertia();

        std::vector<edyn::rigidbody_def> defs;

        // Instantiate multiple links to create a chain.
        for (size_t i = 0; i < 6; ++i) {
            auto j = static_cast<edyn::scalar>(i % 2 + 1);
            auto k = static_cast<edyn::scalar>(i) * 0.725f;
            dyn_def.position = {k, 0.6, 0.0};
            dyn_def.orientation = edyn::quaternion_axis_angle({1, 0, 0}, j * edyn::pi * 0.5);
            defs.push_back(dyn_def);
        }

        edyn::batch_rigidbodies(*m_registry, defs);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleCompound
    , "04-compound"
    , "Compound Shapes."
    , "https://github.com/xissburg/edyn"
    );
