#include <gtest/gtest.h>

#include <frame_deadline_monitor.hpp>

namespace
{

using fast_lio::diagnostics::ClassifyDominantTime;
using fast_lio::diagnostics::DeadlineExceeded;
using fast_lio::diagnostics::DominantTime;
using fast_lio::diagnostics::DominantTimeName;
using fast_lio::diagnostics::MillisecondsToNanoseconds;
using fast_lio::diagnostics::FrameDeadlineMonitor;
using fast_lio::diagnostics::Stage;
using fast_lio::diagnostics::StageDuration;
using fast_lio::diagnostics::TimingSnapshot;
using fast_lio::diagnostics::WallMinusMainThreadCpuNanoseconds;
using fast_lio::diagnostics::WarningThrottle;

TEST(DeadlineMonitor, UsesStrictFiftyMillisecondBoundary)
{
    const std::uint64_t deadline = MillisecondsToNanoseconds(50.0);
    TimingSnapshot snapshot;

    snapshot.total_wall_ns = MillisecondsToNanoseconds(49.999);
    EXPECT_FALSE(DeadlineExceeded(snapshot, deadline));

    snapshot.total_wall_ns = MillisecondsToNanoseconds(50.0);
    EXPECT_FALSE(DeadlineExceeded(snapshot, deadline));

    snapshot.total_wall_ns = MillisecondsToNanoseconds(50.001);
    EXPECT_TRUE(DeadlineExceeded(snapshot, deadline));
}

TEST(DeadlineMonitor, ClassifiesMainThreadCpuAndParallelOrWaitConservatively)
{
    TimingSnapshot snapshot;
    snapshot.thread_cpu_valid = true;
    snapshot.total_wall_ns = MillisecondsToNanoseconds(100.0);
    snapshot.total_thread_cpu_ns = MillisecondsToNanoseconds(80.0);
    EXPECT_EQ(DominantTime::MainThreadCpu, ClassifyDominantTime(snapshot));
    EXPECT_STREQ("main_thread_cpu", DominantTimeName(ClassifyDominantTime(snapshot)));

    snapshot.total_wall_ns = MillisecondsToNanoseconds(900.0);
    snapshot.total_thread_cpu_ns = MillisecondsToNanoseconds(30.0);
    EXPECT_EQ(DominantTime::ParallelOrWait, ClassifyDominantTime(snapshot));
    EXPECT_STREQ("parallel_or_wait", DominantTimeName(ClassifyDominantTime(snapshot)));
}

TEST(DeadlineMonitor, ClampsWallMinusMainThreadCpuWhenCpuClockIsLarger)
{
    TimingSnapshot snapshot;
    snapshot.thread_cpu_valid = true;
    snapshot.total_wall_ns = MillisecondsToNanoseconds(10.0);
    snapshot.total_thread_cpu_ns = MillisecondsToNanoseconds(10.001);
    EXPECT_EQ(0ULL, WallMinusMainThreadCpuNanoseconds(snapshot));
}

TEST(DeadlineMonitor, ReportsUnknownWhenThreadCpuClockIsUnavailable)
{
    TimingSnapshot snapshot;
    snapshot.thread_cpu_valid = false;
    snapshot.total_wall_ns = MillisecondsToNanoseconds(100.0);
    EXPECT_EQ(DominantTime::Unknown, ClassifyDominantTime(snapshot));
    EXPECT_STREQ("unknown", DominantTimeName(ClassifyDominantTime(snapshot)));
}

TEST(DeadlineMonitor, ThrottlesRepeatedSlowFramesUsingSteadyTime)
{
    WarningThrottle throttle(MillisecondsToNanoseconds(1000.0));
    const std::uint64_t start = MillisecondsToNanoseconds(5000.0);

    EXPECT_TRUE(throttle.Allow(start));
    for (int i = 1; i < 1000; ++i)
    {
        EXPECT_FALSE(throttle.Allow(start + MillisecondsToNanoseconds(i * 0.9)));
    }
    EXPECT_TRUE(throttle.Allow(start + MillisecondsToNanoseconds(1000.0)));
}

TEST(DeadlineMonitor, ThrottleRecoversIfItsInputClockMovesBackward)
{
    WarningThrottle throttle(MillisecondsToNanoseconds(1000.0));
    EXPECT_TRUE(throttle.Allow(MillisecondsToNanoseconds(5000.0)));
    EXPECT_TRUE(throttle.Allow(MillisecondsToNanoseconds(4000.0)));
    EXPECT_FALSE(throttle.Allow(MillisecondsToNanoseconds(4500.0)));
}

TEST(DeadlineMonitor, StageDurationsCoverTheFrameAndFinishIsIdempotent)
{
    FrameDeadlineMonitor monitor;
    monitor.Mark(Stage::Imu);
    monitor.Mark(Stage::Icp);
    const TimingSnapshot first = monitor.Finish(Stage::Other);
    const TimingSnapshot second = monitor.Finish(Stage::MapUpdate);

    std::uint64_t summed_wall_ns = 0;
    std::uint64_t summed_main_thread_cpu_ns = 0;
    for (std::size_t i = 0; i < static_cast<std::size_t>(Stage::Count); ++i)
    {
        const auto &duration = first.stages[i];
        summed_wall_ns += duration.wall_ns;
        summed_main_thread_cpu_ns += duration.thread_cpu_ns;
    }

    EXPECT_EQ(first.total_wall_ns, summed_wall_ns);
    if (first.thread_cpu_valid)
    {
        EXPECT_EQ(first.total_thread_cpu_ns, summed_main_thread_cpu_ns);
    }
    EXPECT_EQ(first.total_wall_ns, second.total_wall_ns);
    EXPECT_EQ(first.total_thread_cpu_ns, second.total_thread_cpu_ns);
    EXPECT_EQ(
        StageDuration(first, Stage::MapUpdate).wall_ns,
        StageDuration(second, Stage::MapUpdate).wall_ns);
}

TEST(DeadlineMonitor, DisabledDeadlineNeverReports)
{
    TimingSnapshot snapshot;
    snapshot.total_wall_ns = MillisecondsToNanoseconds(1000.0);
    EXPECT_FALSE(DeadlineExceeded(snapshot, 0));
}

}  // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
