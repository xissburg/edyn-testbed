#include "edyn_example.hpp"

#ifdef EDYN_SOUND_ENABLED
#include <soloud_wav.h>
#endif

class ExampleBilliards : public EdynExample
{
public:
    ExampleBilliards(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {

    }

    virtual ~ExampleBilliards() {}

    void createScene() override
    {
        // Material properties obtained from
        // https://billiards.colostate.edu/faq/physics/physical-properties/
        auto ball_diameter = edyn::scalar(0.05715);
        auto ball_radius = edyn::scalar(ball_diameter / 2);

        auto ball_ball_mat = edyn::material_base{};
        ball_ball_mat.friction = 0.05;
        ball_ball_mat.restitution = 0.95;
        edyn::insert_material_mixing(*m_registry, m_ball_mat_id, m_ball_mat_id, ball_ball_mat);

        auto table_ball_mat = edyn::material_base{};
        table_ball_mat.friction = 0.2;
        table_ball_mat.restitution = 0.5;
        table_ball_mat.spin_friction = 0.000057;
        // Multiply rolling resistance by the ball radius because in Edyn the
        // rolling friction applies torque.
        table_ball_mat.roll_friction = 0.006 * ball_radius;
        edyn::insert_material_mixing(*m_registry, m_ball_mat_id, m_table_mat_id, table_ball_mat);

        auto rail_ball_mat = edyn::material_base{};
        rail_ball_mat.friction = 0.2;
        rail_ball_mat.restitution = 0.7;
        edyn::insert_material_mixing(*m_registry, m_ball_mat_id, m_rail_mat_id, rail_ball_mat);

        // Create floor
        auto floor_def = edyn::rigidbody_def();
        floor_def.kind = edyn::rigidbody_kind::rb_static;
        floor_def.material->restitution = 0.1;
        floor_def.material->friction = 0.2;
        floor_def.shape = edyn::plane_shape{{0, 1, 0}, 0};
        edyn::make_rigidbody(*m_registry, floor_def);

        // Create table.
        auto table_size = edyn::vector3{1.268, 0.76, 2.385};
        auto table_def = edyn::rigidbody_def();
        table_def.kind = edyn::rigidbody_kind::rb_static;
        table_def.material->id = m_table_mat_id;
        table_def.material->restitution = 0.5;
        table_def.material->friction = 0.2;
        table_def.shape = edyn::box_shape{table_size / 2};
        table_def.position = {0, table_size.y / 2, 0};
        edyn::make_rigidbody(*m_registry, table_def);

        // Rail top.
        auto rail_def = edyn::rigidbody_def();
        rail_def.kind = edyn::rigidbody_kind::rb_static;
        rail_def.material->id = m_rail_mat_id;
        rail_def.material->restitution = 0.7;
        rail_def.material->friction = 0.2;
        rail_def.shape = edyn::box_shape{table_size.x / 2, 0.025, 0.075};
        rail_def.position = {0, table_size.y + 0.025f, table_size.z / 2.f - 0.075f};
        edyn::make_rigidbody(*m_registry, rail_def);

        // Rail bottom.
        rail_def.position.z *= -1;
        edyn::make_rigidbody(*m_registry, rail_def);

        // Rail left.
        rail_def.shape = edyn::box_shape{0.075, 0.025, table_size.z / 2};
        rail_def.position = {table_size.x / 2.f - 0.075f, table_size.y + 0.025f, 0};
        edyn::make_rigidbody(*m_registry, rail_def);

        // Rail right.
        rail_def.position.x *= -1;
        edyn::make_rigidbody(*m_registry, rail_def);

        // Add the balls.
        auto def = edyn::rigidbody_def();
        def.mass = 0.17;
        def.material->id = m_ball_mat_id;
        def.material->friction = 0.2;
        def.material->restitution = 0.95;
        def.shape = edyn::sphere_shape{ball_radius};
        def.update_inertia();
        def.continuous_contacts = true;

        std::vector<edyn::rigidbody_def> defs;

        // Cue ball.
        def.position = {0, table_size.y + ball_radius, -(table_size.z / 2 - 0.15f) / 2};
        def.linvel = {0.001, 0, 3};
        def.angvel = {0, -3, 1};
        defs.push_back(def);

        // Other balls.
        def.linvel = {0, 0, 0};
        def.angvel = {0, 0, 0};

        for (auto i = 0; i < 5; ++i) {
            auto n = i + 1;
            def.position.z = float(i) * ball_diameter * std::sin(edyn::to_radians(60)) +
                             (table_size.z / 2 - 0.15f) / 2;

            for (auto j = 0; j < n; ++j) {
                def.position.x = (j - float(i) / 2) * ball_diameter;
                defs.push_back(def);
            }
        }

        edyn::batch_rigidbodies(*m_registry, defs);

        cameraSetPosition({0.0f, 1.6f, -2.f});
        cameraSetVerticalAngle(-0.25f);

        m_rigid_body_axes_size = ball_radius + 0.003f;
        setPaused(true);

#ifdef EDYN_SOUND_ENABLED
        // Create sounds
        // Source: https://freesound.org/people/juskiddink/sounds/108615/
        m_ball_ball_collision_sound.load("../../../edyn-testbed/resources/108615__juskiddink__billiard-balls-single-hit-dry.wav");
        m_ball_ball_collision_sound.set3dMinMaxDistance(0, 30);

        // Source: https://freesound.org/people/Reitanna/sounds/332661/
        m_ball_table_collision_sound.load("../../../edyn-testbed/resources/332661__reitanna__big-thud.wav");
        m_ball_table_collision_sound.set3dMinMaxDistance(0, 30);

        // Register contact event handlers.
        edyn::on_contact_started(*m_registry).connect<&ExampleBilliards::onContactStarted>(*this);
#endif
    }

#ifdef EDYN_SOUND_ENABLED
    void onContactStarted(entt::entity manifold_entity) {
        auto &manifold = m_registry->get<edyn::contact_manifold>(manifold_entity);
        auto &materialA = m_registry->get<edyn::material>(manifold.body[0]);
        auto &materialB = m_registry->get<edyn::material>(manifold.body[1]);

        auto is_ball_ball = materialA.id == m_ball_mat_id && materialB.id == m_ball_mat_id;
        auto is_ball_table =
            (materialA.id == m_ball_mat_id && (materialB.id == m_table_mat_id || materialB.id == m_rail_mat_id)) ||
            (materialB.id == m_ball_mat_id && (materialA.id == m_table_mat_id || materialA.id == m_rail_mat_id));

        auto &cp = manifold.point[manifold.ids[0]];
        auto &posA = m_registry->get<edyn::position>(manifold.body[0]);
        auto &ornA = m_registry->get<edyn::orientation>(manifold.body[0]);
        auto pos_cp = edyn::to_world_space(cp.pivotA, posA, ornA);

        if (is_ball_ball) {
            auto volume = (cp.normal_impulse + cp.normal_restitution_impulse) * 20.f;
            auto handle = m_soloud.play3d(m_ball_ball_collision_sound,
                                          pos_cp.x, pos_cp.y, pos_cp.z,
                                          0, 0, 0, volume);
            m_soloud.set3dSourceAttenuation(handle, SoLoud::AudioSource::LINEAR_DISTANCE, 1);
        } else if (is_ball_table) {
            auto volume = (cp.normal_impulse + cp.normal_restitution_impulse) * 1.1f;
            auto handle = m_soloud.play3d(m_ball_table_collision_sound,
                                          pos_cp.x, pos_cp.y, pos_cp.z,
                                          0, 0, 0, volume);
            m_soloud.set3dSourceAttenuation(handle, SoLoud::AudioSource::LINEAR_DISTANCE, 1);
        }
    }
#endif

    const edyn::material::id_type m_ball_mat_id = 0;
    const edyn::material::id_type m_table_mat_id = 1;
    const edyn::material::id_type m_rail_mat_id = 2;

#ifdef EDYN_SOUND_ENABLED
    SoLoud::Wav m_ball_ball_collision_sound;
    SoLoud::Wav m_ball_table_collision_sound;
#endif
};

ENTRY_IMPLEMENT_MAIN(
    ExampleBilliards
    , "17-billiards"
    , "Billiards with sound effects."
    , "https://github.com/xissburg/edyn-testbed"
    );
