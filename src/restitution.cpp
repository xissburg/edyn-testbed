#include "edyn_example.hpp"

class ExampleRestitution : public EdynExample
{
public:
	ExampleRestitution(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

    virtual ~ExampleRestitution() {}

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

        // Add some bouncy spheres.
        auto def = edyn::rigidbody_def();
        def.friction = 0.8;
        def.mass = 100;
        def.shape_opt = {edyn::sphere_shape{0.2}};
        def.update_inertia();
        def.continuous_contacts = true;

        const size_t n = 10;
        for (size_t i = 0; i < n; ++i) {
            def.restitution = edyn::scalar(i) / n;
            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * edyn::scalar(0.8), 5, 0};
            edyn::make_rigidbody(*m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * edyn::scalar(0.8), 6, 0};
            edyn::make_rigidbody(*m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * edyn::scalar(0.8), 7, 0};
            edyn::make_rigidbody(*m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * edyn::scalar(0.8), 8, 0};
            edyn::make_rigidbody(*m_registry, def);

            def.position = {(edyn::scalar(i) - edyn::scalar(n)/2) * edyn::scalar(0.8), 9, 0};
            edyn::make_rigidbody(*m_registry, def);
        }
	}
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleRestitution
	, "08-restitution"
	, "Restitution."
    , "https://github.com/xissburg/edyn-testbed"
	);
