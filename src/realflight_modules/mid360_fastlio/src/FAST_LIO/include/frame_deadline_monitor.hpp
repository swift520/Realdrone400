#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>

namespace fast_lio
{
namespace diagnostics
{

constexpr std::uint64_t kNanosecondsPerMillisecond = 1000000ULL;

enum class Stage : std::size_t
{
    Callbacks = 0,
    Imu,
    Fov,
    Downsample,
    PreIcp,
    Icp,
    PostIcp,
    OdomPublish,
    MapUpdate,
    OutputPublish,
    RuntimeLog,
    Other,
    Count
};

struct DurationPair
{
    std::uint64_t wall_ns = 0;
    std::uint64_t thread_cpu_ns = 0;
};

struct TimingSnapshot
{
    std::array<DurationPair, static_cast<std::size_t>(Stage::Count)> stages{};
    std::uint64_t total_wall_ns = 0;
    std::uint64_t total_thread_cpu_ns = 0;
    std::uint64_t end_wall_ns = 0;
    bool thread_cpu_valid = true;
};

inline std::uint64_t MillisecondsToNanoseconds(double milliseconds)
{
    return milliseconds <= 0.0
               ? 0ULL
               : static_cast<std::uint64_t>(
                     milliseconds * static_cast<double>(kNanosecondsPerMillisecond));
}

inline double NanosecondsToMilliseconds(std::uint64_t nanoseconds)
{
    return static_cast<double>(nanoseconds) /
           static_cast<double>(kNanosecondsPerMillisecond);
}

inline bool DeadlineExceeded(
    const TimingSnapshot &snapshot, std::uint64_t deadline_ns)
{
    return deadline_ns > 0 && snapshot.total_wall_ns > deadline_ns;
}

inline std::uint64_t WallMinusMainThreadCpuNanoseconds(
    const TimingSnapshot &snapshot)
{
    return snapshot.total_wall_ns > snapshot.total_thread_cpu_ns
               ? snapshot.total_wall_ns - snapshot.total_thread_cpu_ns
               : 0ULL;
}

enum class DominantTime
{
    MainThreadCpu,
    ParallelOrWait,
    Unknown
};

inline DominantTime ClassifyDominantTime(const TimingSnapshot &snapshot)
{
    if (!snapshot.thread_cpu_valid)
    {
        return DominantTime::Unknown;
    }
    return WallMinusMainThreadCpuNanoseconds(snapshot) >
                   snapshot.total_thread_cpu_ns
               ? DominantTime::ParallelOrWait
               : DominantTime::MainThreadCpu;
}

inline const char *DominantTimeName(DominantTime dominant)
{
    switch (dominant)
    {
    case DominantTime::MainThreadCpu:
        return "main_thread_cpu";
    case DominantTime::ParallelOrWait:
        return "parallel_or_wait";
    default:
        return "unknown";
    }
}

class WarningThrottle
{
public:
    explicit WarningThrottle(std::uint64_t interval_ns)
        : interval_ns_(interval_ns)
    {
    }

    bool Allow(std::uint64_t now_ns)
    {
        if (!has_previous_ || now_ns < previous_ns_ || interval_ns_ == 0 ||
            now_ns - previous_ns_ >= interval_ns_)
        {
            previous_ns_ = now_ns;
            has_previous_ = true;
            return true;
        }
        return false;
    }

private:
    std::uint64_t interval_ns_ = 0;
    std::uint64_t previous_ns_ = 0;
    bool has_previous_ = false;
};

class FrameDeadlineMonitor
{
public:
    FrameDeadlineMonitor()
        : start_(SampleNow()), last_(start_)
    {
        snapshot_.thread_cpu_valid = start_.thread_cpu_valid;
    }

    void Mark(Stage stage)
    {
        if (finished_)
        {
            return;
        }
        const ClockSample now = SampleNow();
        Accumulate(stage, now);
    }

    TimingSnapshot Finish(Stage trailing_stage = Stage::Other)
    {
        if (!finished_)
        {
            const ClockSample now = SampleNow();
            Accumulate(trailing_stage, now);
            snapshot_.total_wall_ns = Elapsed(now.wall_ns, start_.wall_ns);
            if (snapshot_.thread_cpu_valid && now.thread_cpu_valid)
            {
                snapshot_.total_thread_cpu_ns =
                    Elapsed(now.thread_cpu_ns, start_.thread_cpu_ns);
            }
            else
            {
                snapshot_.thread_cpu_valid = false;
                snapshot_.total_thread_cpu_ns = 0;
            }
            snapshot_.end_wall_ns = now.wall_ns;
            finished_ = true;
        }
        return snapshot_;
    }

    static std::uint64_t SteadyNowNanoseconds()
    {
        return SampleWallNow();
    }

private:
    struct ClockSample
    {
        std::uint64_t wall_ns = 0;
        std::uint64_t thread_cpu_ns = 0;
        bool thread_cpu_valid = false;
    };

    static std::uint64_t Elapsed(std::uint64_t end_ns, std::uint64_t begin_ns)
    {
        return end_ns >= begin_ns ? end_ns - begin_ns : 0ULL;
    }

    static std::uint64_t SampleWallNow()
    {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    static ClockSample SampleNow()
    {
        ClockSample sample;
        sample.wall_ns = SampleWallNow();

#ifdef CLOCK_THREAD_CPUTIME_ID
        timespec thread_cpu_time{};
        if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &thread_cpu_time) == 0)
        {
            sample.thread_cpu_ns =
                static_cast<std::uint64_t>(thread_cpu_time.tv_sec) * 1000000000ULL +
                static_cast<std::uint64_t>(thread_cpu_time.tv_nsec);
            sample.thread_cpu_valid = true;
        }
#endif
        return sample;
    }

    void Accumulate(Stage stage, const ClockSample &now)
    {
        DurationPair &duration = stagesAt(stage);
        duration.wall_ns += Elapsed(now.wall_ns, last_.wall_ns);
        if (snapshot_.thread_cpu_valid && last_.thread_cpu_valid &&
            now.thread_cpu_valid)
        {
            duration.thread_cpu_ns +=
                Elapsed(now.thread_cpu_ns, last_.thread_cpu_ns);
        }
        else
        {
            snapshot_.thread_cpu_valid = false;
        }
        last_ = now;
    }

    DurationPair &stagesAt(Stage stage)
    {
        return snapshot_.stages[static_cast<std::size_t>(stage)];
    }

    ClockSample start_;
    ClockSample last_;
    TimingSnapshot snapshot_;
    bool finished_ = false;
};

inline const DurationPair &StageDuration(
    const TimingSnapshot &snapshot, Stage stage)
{
    return snapshot.stages[static_cast<std::size_t>(stage)];
}

}  // namespace diagnostics
}  // namespace fast_lio
