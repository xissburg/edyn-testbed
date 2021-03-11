#include "edyn_example.hpp"

void on_construct_contact_point(entt::registry &registry, entt::entity entity) {
    registry.emplace<edyn::continuous>(entity).insert<edyn::contact_point>();
    registry.emplace<edyn::dirty>(entity).created<edyn::continuous>();
}

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
        // Create entities.
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 0;
        floor_def.friction = 0.5;
        //floor_def.orientation = edyn::quaternion_axis_angle({0,0,1}, edyn::to_radians(10));
        //floor_def.shape_opt = {edyn::box_shape{{2, 0.5, 2}}};// {edyn::plane_shape{{0, 1, 0}, 0}};
        floor_def.shape_opt = {edyn::plane_shape{{0, 1, 0}, 0}};
        floor_def.presentation = true;
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.presentation = true;
        def.friction = 0.8;
        def.mass = 10;
        def.restitution = 0;
        def.position = {0, 4, 0};
        //def.orientation = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi * 0.0);
        //def.shape_opt = {edyn::sphere_shape{0.2}};
        def.shape_opt = {edyn::box_shape{0.2, 0.2, 0.2}};
        //def.gravity = {0,0,0};
        def.update_inertia();

    #if 0
        const auto n = 3;

        for (int i = 0; i < n; ++i) {
            //def.position = {edyn::scalar(-0.6 + i * 1.2), 1.8, 0};
            def.position = {edyn::scalar(0.6 + i * 0.99), 0.6, 0};
            if (i == 1) {
                def.position.x += 0.01;
                def.position.z -= 0.01;
            }
            edyn::make_rigidbody(*m_registry, def);
        }
    #else
        /* for (int i = 0; i < 10; ++i) {
            for (int j = 0; j < 10; ++j) {
                //def.position = {edyn::scalar(-0.6 + i * 1.2), 1.8, 0};
                def.position = {edyn::scalar(-5 + j), edyn::scalar(0.6 + i * 0.6), 0};
                if (i == 1) {
                    def.position.x += 0.01;
                    def.position.z -= 0.01;
                }
                edyn::make_rigidbody(*m_registry, def);
            }
        } */

        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                for (int k = 0; k < 5; ++k) {
                    for (int l = 0; l < 1; ++l) {

                    def.position = {edyn::scalar(0.4 * j + l * 7), edyn::scalar(0.6 + 0.4 * i), edyn::scalar(0.4 * k)};
                    edyn::make_rigidbody(*m_registry, def);
                    }
                }
            }
        }
    #endif

        m_registry->on_construct<edyn::contact_point>().connect<&on_construct_contact_point>();

        m_pause = true;
        auto& world = m_registry->ctx<edyn::world>();
        world.set_paused(m_pause);
	}
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleBoxes
	, "00-boxes"
	, "Boxes."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
	);


