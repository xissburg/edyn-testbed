#include <edyn/context/task.hpp>
#include <enkiTS/TaskScheduler.h>
#include "edyn_example.hpp"

enki::TaskScheduler g_TS;

struct CompletionActionDelete : public enki::ICompletable
{
    enki::Dependency m_dependency;
    edyn::task_completion_delegate_t m_completion;

    void OnDependenciesComplete(enki::TaskScheduler* scheduler, uint32_t threadNum)
    {
        if (m_completion) {
            m_completion();
        }

        enki::ICompletable::OnDependenciesComplete(scheduler, threadNum);
        delete m_dependency.GetDependencyTask();
    }
};

struct DelegateWithCompletionTaskSet : public enki::ITaskSet {
    CompletionActionDelete m_task_deleter;
    enki::Dependency m_dependency;
    edyn::task_delegate_t m_task;

    DelegateWithCompletionTaskSet(uint32_t size, uint32_t grain) : enki::ITaskSet(size, grain)
    {
        m_task_deleter.SetDependency(m_task_deleter.m_dependency, this);
    }

    void ExecuteRange(enki::TaskSetPartition range, uint32_t threadnum) override {
        m_task(range.start, range.end);
    }
};

struct DelegateTaskSet : public enki::ITaskSet {
    edyn::task_delegate_t m_task;

    DelegateTaskSet(uint32_t size, uint32_t grain) : enki::ITaskSet(size, grain) {}

    void ExecuteRange(enki::TaskSetPartition range, uint32_t threadnum) override {
        m_task(range.start, range.end);
    }
};

class ExampleEnkiTS : public EdynExample
{
public:
    ExampleEnkiTS(const char* _name, const char* _description, const char* _url)
        : EdynExample(_name, _description, _url)
    {
    }

    void init(int32_t _argc, const char* const* _argv, uint32_t _width, uint32_t _height) override
    {
        g_TS.Initialize();
        EdynExample::init(_argc, _argv, _width, _height);
    }

    int shutdown() override
    {
        auto ret = EdynExample::shutdown();
        g_TS.WaitforAllAndShutdown();
        return ret;
    }

    void initEdyn() override
    {
        auto config = edyn::init_config{};
        config.execution_mode = edyn::execution_mode::asynchronous;
        config.enqueue_task = [](edyn::task_delegate_t task, unsigned size, edyn::task_completion_delegate_t completion) {
            auto grain_size = std::max(size / g_TS.GetNumTaskThreads(), 1u);
            auto task_set = new DelegateWithCompletionTaskSet(size, grain_size);
            task_set->m_task = std::move(task);
            task_set->m_task_deleter.m_completion = std::move(completion);
            g_TS.AddTaskSetToPipe(task_set);
        };
        config.enqueue_task_wait = [](edyn::task_delegate_t task, unsigned size) {
            auto grain_size = std::max(size / g_TS.GetNumTaskThreads(), 1u);
            DelegateTaskSet task_set(size, grain_size);
            task_set.m_task = std::move(task);
            g_TS.AddTaskSetToPipe(&task_set);
            g_TS.WaitforTask(&task_set);
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
    ExampleEnkiTS
    , "33-enkiTS"
    , "enkiTS."
    , "https://github.com/xissburg/edyn-testbed"
    );
