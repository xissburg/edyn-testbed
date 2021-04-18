#include "edyn_example.hpp"
#include <edyn/constraints/constraint.hpp>
#include <edyn/math/quaternion.hpp>
#include <edyn/util/moment_of_inertia.hpp>

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
        floor_def.restitution = 1;
        floor_def.friction = 0.5;
        floor_def.shape_opt = {edyn::plane_shape{{0, 1, 0}, 0}};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Create a chain link using multiple shapes.
        // Increasing `max_contacts` in "edyn/config/constants.hpp" will yield
        // more stable results and lower likelihood of separation.
        auto compound = edyn::compound_shape{};
        compound.add_shape(edyn::box_shape{0.05, 0.05, 0.25}, {-0.45, 0, 0}, edyn::quaternion_identity);
        compound.add_shape(edyn::box_shape{0.05, 0.05, 0.25}, { 0.0, 0, 0}, edyn::quaternion_identity);
        compound.add_shape(edyn::box_shape{0.05, 0.05, 0.25}, { 0.45, 0, 0}, edyn::quaternion_identity);
        compound.add_shape(edyn::box_shape{ 0.5, 0.05, 0.05}, { 0.0, 0, 0.2}, edyn::quaternion_identity);
        compound.add_shape(edyn::box_shape{ 0.5, 0.05, 0.05}, { 0.0, 0, -0.2}, edyn::quaternion_identity);
        compound.finish();

        auto dyn_def = edyn::rigidbody_def();
        dyn_def.kind = edyn::rigidbody_kind::rb_dynamic;
        dyn_def.continuous_contacts = true;
        dyn_def.mass = 50;
        dyn_def.shape_opt = {compound};
        dyn_def.inertia = edyn::diagonal_matrix(edyn::moment_of_inertia_solid_box(dyn_def.mass, {1, 0.1, 0.5}));

        // Create multiple links to create a chain.
        for (size_t i = 0; i < 8; ++i) {
            auto j = static_cast<edyn::scalar>(i % 2);
            auto k = static_cast<edyn::scalar>(i) * 0.725f;
            dyn_def.position = {k, 1.5, 0.0};
            dyn_def.orientation = edyn::quaternion_axis_angle({1, 0, 0}, j * edyn::pi * 0.5);
            edyn::make_rigidbody(*m_registry, dyn_def);
        }

        m_pause = true;
        auto& world = m_registry->ctx<edyn::world>();
        world.set_paused(m_pause);
    }
};

ENTRY_IMPLEMENT_MAIN(
    ExampleCompound
    , "00-compound"
    , "Compound Shapes."
    , "https://bkaradzic.github.io/bgfx/examples.html#cubes"
    );


