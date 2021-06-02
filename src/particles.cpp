#include "edyn_example.hpp"

class ExampleParticles : public EdynExample
{
public:
	ExampleParticles(const char* _name, const char* _description, const char* _url)
		: EdynExample(_name, _description, _url)
	{

	}

	void createScene() override
    {
        // Create a few particle entities.
        std::vector<entt::entity> entities;

        auto def = edyn::rigidbody_def();
        def.gravity = edyn::vector3_zero;

        def.position = {3, 3, 0};
        def.linvel = {0, 4, -1};
        def.angvel = {1, 3, 0.1};
        def.mass = 100;
        def.inertia = edyn::diagonal_matrix({40, 40, 40});
        entities.push_back(edyn::make_rigidbody(*m_registry, def));

        def.position = {-7, 3.2, 4.2};
        def.linvel = {0, 2.1, 0};
        def.angvel = edyn::vector3_zero;
        def.mass = 1e10;
        def.inertia = edyn::diagonal_matrix({1e9, 1e9, 1e9});
        entities.push_back(edyn::make_rigidbody(*m_registry, def));

        def.position = {0, 3, 0};
        def.linvel = {0, 0, 0};
        def.angvel = edyn::vector3_zero;
        def.mass = 1e12;
        def.inertia = edyn::diagonal_matrix({1e11, 1e11, 1e11});
        entities.push_back(edyn::make_rigidbody(*m_registry, def));

        for (size_t i = 0; i < entities.size(); ++i) {
            for (size_t j = i + 1; j < entities.size(); ++j) {
                edyn::make_constraint<edyn::gravity_constraint>(*m_registry, entities[i], entities[j]);
            }
        }
	}
};

ENTRY_IMPLEMENT_MAIN(
	  ExampleParticles
	, "06-particles"
	, "Particles orbiting one another with gravity."
    , "https://github.com/xissburg/edyn-testbed"
	);
