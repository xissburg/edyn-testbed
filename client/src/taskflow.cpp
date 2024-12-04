#include "edyn_example.hpp"
#include <taskflow/core/declarations.hpp>
#include <taskflow/taskflow.hpp>
#include <taskflow/algorithm/for_each.hpp>

tf::Executor *g_executor {nullptr};

class ExampleTaskflow : public EdynExample
{
public:
    ExampleTaskflow(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {
    }

    void init(int32_t _argc, const char* const* _argv, uint32_t _width, uint32_t _height) override
    {
        g_executor = new tf::Executor;
        EdynExample::init(_argc, _argv, _width, _height);
    }

    int shutdown() override
    {
        auto ret = EdynExample::shutdown();
        delete g_executor;
        g_executor = nullptr;
        return ret;
    }

    void initEdyn() override
    {

        auto config = edyn::init_config{};
        config.execution_mode = edyn::execution_mode::asynchronous;
        config.enqueue_task = [](edyn::task_delegate_t task, unsigned size, edyn::task_completion_delegate_t completion) {
            tf::Taskflow taskflow;
            auto taskA = taskflow.for_each_index(0u, size, 1u, [task](unsigned i) {
                task(i, i + 1);
            });

            if (completion) {
                auto taskB = taskflow.emplace([completion]() { completion(); });
                taskA.precede(taskB);
            }

            g_executor->run(std::move(taskflow));
        };
        config.enqueue_task_wait = [](edyn::task_delegate_t task, unsigned size) {
            tf::Taskflow taskflow;
            taskflow.for_each_index(0u, size, 1u, [task](unsigned i) { task(i, i + 1); });
            g_executor->run(taskflow).wait();
        };
        edyn::attach(*m_registry, config);
    }

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

        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                for (int k = 0; k < 5; ++k) {
                    def.position = {edyn::scalar(0.4 * j),
                                    edyn::scalar(0.4 * i + 0.6),
                                    edyn::scalar(0.4 * k)};
                    edyn::make_rigidbody(*m_registry, def);
                }
            }
        }
    }

};

ENTRY_IMPLEMENT_MAIN(
    ExampleTaskflow
    , "32-taskflow"
    , "Taskflow."
    , "https://github.com/xissburg/edyn-testbed"
    );
