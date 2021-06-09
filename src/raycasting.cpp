#include "edyn_example.hpp"
#include <edyn/collision/raycast.hpp>

class ExampleRaycasting : public EdynExample
{
public:
	ExampleRaycasting(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

    virtual ~ExampleRaycasting() {}

	void createScene() override
	{
        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.restitution = 0;
        floor_def.friction = 0.5;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Add some boxes.
        auto def = edyn::rigidbody_def();
        def.friction = 0.8;
        def.mass = 10;
        def.restitution = 0;
        def.shape = edyn::polyhedron_shape("../../../edyn-testbed/resources/rock.obj");
        def.update_inertia();
        def.continuous_contacts = true;

        std::vector<edyn::rigidbody_def> defs;

        for (int i = 0; i < 1; ++i) {
            for (int j = 0; j < 1; ++j) {
                for (int k = 0; k < 1; ++k) {
                    def.position = {edyn::scalar(0.4 * j),
                                    edyn::scalar(0.4 * i + 0.6),
                                    edyn::scalar(0.4 * k)};
                    defs.push_back(def);
                }
            }
        }

        edyn::batch_rigidbodies(*m_registry, defs);
	}

    void updatePhysics(float deltaTime) override {
        EdynExample::updatePhysics(deltaTime);

        auto p0 = edyn::vector3{cameraGetPosition().x, cameraGetPosition().y, cameraGetPosition().z};
        auto p1 = p0 + m_rayDir * 100.f;

        auto result = edyn::raycast(*m_registry, p0, p1);

        if (result.entity != entt::null) {
            m_registry->emplace_or_replace<edyn::shape_raycast_result>(result.entity, result);
        } else {
            m_registry->clear<edyn::shape_raycast_result>();
        }
    }
};

ENTRY_IMPLEMENT_MAIN(
	ExampleRaycasting
	,"00-raycasting"
	, "Ray-casting."
    , "https://github.com/xissburg/edyn-testbed"
    );
