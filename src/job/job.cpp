#include "job/job.h"
#include "job/binary_io.h"
#include "job/job_file.h"
#include "job/thread_pool.h"
#include "nav/area_builder.h"
#include "nav/nav_file.h"
#include "nav/sampling.h"

#include <algorithm>
#include <chrono>

namespace navfast {
namespace {

using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double>(end - start).count();
}

void validate(const JobInputs& inputs) {
    const auto& params = inputs.params;
    if (!std::isfinite(params.slopeLimit) || params.slopeLimit < -1 || params.slopeLimit > 1 ||
        !std::isfinite(params.slopeTolerance) || params.areaMaxSize <= 0)
        throw std::runtime_error("Invalid generation parameters");
    if (inputs.mode == JobInputs::Generate && inputs.outputPath.empty())
        throw std::runtime_error("Missing .nav output path");
    if (!inputs.dumpPath.empty() && !inputs.outputPath.empty() &&
        std::filesystem::absolute(pathFromUtf8(inputs.dumpPath)).lexically_normal() ==
        std::filesystem::absolute(pathFromUtf8(inputs.outputPath)).lexically_normal())
        throw std::runtime_error(".nav and .navjob paths must differ");
}

} // namespace

Job::~Job() {
    cancel();
    wait();
}

bool Job::running() const {
    return exited_.valid() && exited_.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
}

void Job::start(OwnedJobInputs inputs) {
    if (running()) throw std::runtime_error("A navfast job is already running");
    wait();
    cancelled_ = false;
    std::promise<void> exited;
    exited_ = exited.get_future();
    {
        std::lock_guard lock(mutex_);
        snapshot_ = {.running = true, .phase = "Starting", .message = {}, .error = {}, .stats = {}};
    }
    try {
        runner_ = std::thread([this, inputs = std::move(inputs), exited = std::move(exited)]() mutable {
            lowerThreadPriority();
            run(std::move(inputs));
            exited.set_value_at_thread_exit();
        });
    } catch (...) {
        std::lock_guard lock(mutex_);
        snapshot_ = {};
        throw;
    }
}

JobSnapshot Job::poll() const {
    std::lock_guard lock(mutex_);
    auto snapshot = snapshot_;
    if (snapshot.done && running()) {
        snapshot.running = true;
        snapshot.done = false;
        snapshot.stats.reset();
        snapshot.error.clear();
    }
    return snapshot;
}

void Job::cancel() { cancelled_.store(true, std::memory_order_relaxed); }

std::shared_ptr<const CollisionWorld> Job::world() const {
    std::lock_guard lock(mutex_);
    return world_;
}

void Job::wait() {
    if (runner_.joinable()) runner_.join();
}

void Job::run(OwnedJobInputs owned) {
    const auto started = Clock::now();
    JobStats stats;
    std::string phaseName;
    auto phaseStart = started;
    const auto recordPhase = [&](Clock::time_point now) {
        if (!phaseName.empty()) stats.phases.push_back({phaseName, seconds(phaseStart, now)});
        phaseStart = now;
    };
    const auto phase = [&](std::string_view name) {
        if (cancelled_.load(std::memory_order_relaxed)) throw JobCancelled{};
        recordPhase(Clock::now());
        phaseName = name;
        std::lock_guard lock(mutex_);
        snapshot_.phase = phaseName;
        snapshot_.progress = 0;
        snapshot_.message.clear();
    };
    const auto progress = [&](float fraction, std::string_view message) {
        std::lock_guard lock(mutex_);
        snapshot_.progress = std::clamp(fraction, 0.0f, 1.0f);
        snapshot_.message = message;
    };
    try {
        auto& inputs = owned.inputs;
        inputs.world.bsp = owned.bsp;
        validate(inputs);
        stats.threads = workerCount(inputs.threads);
        if (!inputs.dumpPath.empty()) {
            phase("DumpJob");
            writeJobFile(std::filesystem::absolute(pathFromUtf8(inputs.dumpPath)), inputs);
        }
        phase("BuildCollisionWorld");
        std::shared_ptr<const CollisionWorld> world = buildCollisionWorld(inputs.world);
        stats.worldBounds = world->bounds();
        {
            auto old = world;
            {
                std::lock_guard lock(mutex_);
                old.swap(world_);
            }
        }

        if (inputs.mode == JobInputs::Generate) {
            phase("SampleWalkableSpace");
            ThreadPool pool(stats.threads, cancelled_);
            NavMesh mesh;
            mesh.nodes = sampleWalkableSpace(*world, inputs.seedPositions, inputs.params, pool, progress);
            stats.nodes = mesh.nodes.size();
            if (!stats.nodes) throw std::runtime_error("No walkable ground found at the given seeds");
            phase("CreateAreasFromSamples");
            createNavAreasFromNodes(mesh, *world, inputs.params, pool, progress);
            refineGeneratedAreas(mesh, *world, inputs.params, pool, phase);
            stats.areas = mesh.areaIds().size();
            if (!stats.areas) throw std::runtime_error("No navigation areas could be generated");
            stats.connections = mesh.connectionCount();
            phase("WriteNav");
            writeNav(std::filesystem::absolute(pathFromUtf8(inputs.outputPath)), mesh, binaryCount(owned.bsp.size()));
        } else if (cancelled_.load(std::memory_order_relaxed)) {
            throw JobCancelled{};
        }
        const auto finished = Clock::now();
        recordPhase(finished);
        stats.seconds = seconds(started, finished);
        std::lock_guard lock(mutex_);
        snapshot_ = {.done = true, .phase = "Done", .progress = 1, .message = {}, .error = {}, .stats = std::move(stats)};
    } catch (const std::exception& error) {
        std::lock_guard lock(mutex_);
        snapshot_.running = false;
        snapshot_.done = true;
        snapshot_.error = error.what();
    }
}

} // namespace navfast
