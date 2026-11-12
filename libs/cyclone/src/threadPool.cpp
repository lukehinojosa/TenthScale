#include "threadPool.h"

using namespace cyclone;

ThreadPool::ThreadPool(int threadCount)
{
    if (threadCount <= 0)
    {
        // hardware_concurrency() can return 0 if it doesn't know
        threadCount = (int)std::thread::hardware_concurrency();
        if (threadCount <= 0)
        {
            threadCount = 1;
        }
    }

#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
    // Web builds without pthreads can't start threads, so run() does every job inline
    threadCount = 1;
#endif

    // The thread calling run() is one of them, so start one fewer
    for (int i = 1; i < threadCount; i++)
    {
        m_workers.emplace_back(&ThreadPool::workerLoop, this);
    }
}

ThreadPool::~ThreadPool()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stopping = true;
    }
    m_wake.notify_all();
    for (std::thread& worker : m_workers)
    {
        worker.join();
    }
}

void ThreadPool::run(int jobCount, const std::function<void(int)>& job)
{
    if (jobCount <= 0)
    {
        return;
    }

    // Waking threads costs more than a single job is worth splitting
    if (m_workers.empty() || jobCount == 1)
    {
        for (int i = 0; i < jobCount; i++)
        {
            job(i);
        }
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_job = &job;
        m_jobCount = jobCount;
        m_nextJob = 0;
        m_busyWorkers = (int)m_workers.size();
        m_generation++;
    }
    m_wake.notify_all();

    // Help out instead of waiting idle
    doJobs();

    // Every worker has to check in, even ones that found no jobs left, so
    // none of them is still looking at this run when the next one starts
    std::unique_lock<std::mutex> lock(m_mutex);
    m_done.wait(lock, [this] { return m_busyWorkers == 0; });
    m_job = nullptr;
}

void ThreadPool::workerLoop()
{
    uint64_t seenGeneration = 0;
    for (;;)
    {
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_wake.wait(lock, [&] { return m_stopping || m_generation != seenGeneration; });
            if (m_stopping)
            {
                return;
            }
            seenGeneration = m_generation;
        }

        doJobs();

        std::lock_guard<std::mutex> lock(m_mutex);
        if (--m_busyWorkers == 0)
        {
            m_done.notify_one();
        }
    }
}

void ThreadPool::doJobs()
{
    // Each thread grabs the next untaken job, so a thread that gets quick
    // jobs ends up doing more of them
    for (;;)
    {
        const int i = m_nextJob.fetch_add(1);
        if (i >= m_jobCount)
        {
            return;
        }
        (*m_job)(i);
    }
}
