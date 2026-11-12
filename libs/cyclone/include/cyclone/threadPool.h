#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace cyclone
{
    // Worker threads for splitting a loop across the CPU's cores. They start once and sleep between calls, so
    // run() only wakes them.
    class ThreadPool
    {
    public:
        // threadCount includes the thread that calls run(), which does jobs
        // too. 0 uses one per hardware thread.
        explicit ThreadPool(int threadCount = 0);
        ~ThreadPool();

        ThreadPool(const ThreadPool&) = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;

        // Threads that do jobs during run(), including the calling thread
        int getThreadCount() const { return (int)m_workers.size() + 1; }

        // Runs job once for every index across the threads and returns when all are done. Jobs may run together in
        // any order, so they mustn't touch what another job writes.
        void run(int jobCount, const std::function<void(int)>& job);

    private:
        void workerLoop();

        // Takes jobs until there are none left
        void doJobs();

        std::vector<std::thread> m_workers;

        std::mutex m_mutex;
        std::condition_variable m_wake; // Workers wait on this for a new run
        std::condition_variable m_done; // run() waits on this for the workers to finish
        uint64_t m_generation = 0; // Goes up once per run, so workers can tell a new one has started
        int m_busyWorkers = 0; // Workers still working on the current run
        bool m_stopping = false;

        // The current run's jobs. m_nextJob is the next one nobody has taken yet.
        const std::function<void(int)>* m_job = nullptr;
        int m_jobCount = 0;
        std::atomic<int> m_nextJob{ 0 };
    };
}
