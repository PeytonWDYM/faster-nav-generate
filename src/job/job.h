#pragma once

#include "nav/generation.h"
#include "world/collision_world.h"

#include <atomic>
#include <future>
#include <mutex>
#include <optional>
#include <thread>

namespace navfast {

struct JobInputs {
    WorldInputs world;
    std::vector<Vec3> seedPositions;
    GenerationParams params;
    unsigned threads = 0;
    std::string outputPath;
    std::string dumpPath;
    enum Mode : std::uint8_t { Generate, WorldOnly } mode = Generate;
};

struct OwnedJobInputs {
    std::vector<std::byte> bsp;
    JobInputs inputs;
};

struct PhaseTime {
    std::string name;
    double seconds = 0;
};

struct JobStats {
    std::size_t nodes = 0;
    std::size_t areas = 0;
    std::size_t connections = 0;
    unsigned threads = 0;
    double seconds = 0;
    Aabb worldBounds{};
    std::vector<PhaseTime> phases;
};

struct JobSnapshot {
    bool running = false;
    bool done = false;
    std::string phase;
    float progress = 0;
    std::string message;
    std::string error;
    std::optional<JobStats> stats;
};

/// Owns one background job and the most recently built immutable collision world.
class Job {
public:
    ~Job();

    bool running() const;
    void start(OwnedJobInputs inputs);
    JobSnapshot poll() const;
    void cancel();
    std::shared_ptr<const CollisionWorld> world() const;
    void wait();

private:
    void run(OwnedJobInputs owned);

    mutable std::mutex mutex_;
    JobSnapshot snapshot_;
    std::shared_ptr<const CollisionWorld> world_;
    std::atomic_bool cancelled_{false};
    std::future<void> exited_;
    std::thread runner_;
};

} // namespace navfast
