#pragma once
#include "core/types.hpp"
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
namespace core {
template <class T>
concept Driver = requires(T &driver) {
    { driver.step() } -> std::same_as<void>;
    { driver.period() } -> std::same_as<Period>;
};
struct HostCadence {};
struct SimulationPacing {};
struct TimingSample {
    duration elapsed{}, lateness{};
    std::uint64_t missed_deadlines{};
};
// Optional host pacing around a driver. This utility never manufactures execution time.
template <Driver Target, class Mode>
    requires(std::same_as<Mode, HostCadence> || std::same_as<Mode, SimulationPacing>)
class PacedRunner {
public:
    explicit PacedRunner(Target &target)
        : thread_([this, &target](std::stop_token stop) {
              try {
                  const auto period = target.period().value();
                  auto previous = HostClock::now();
                  auto deadline = previous + period;
                  while (!stop.stop_requested()) {
                      {
                          std::unique_lock lock(mutex_);
                          wake_.wait_until(lock, stop, deadline, [] { return false; });
                      }
                      if (stop.stop_requested())
                          break;
                      const auto now = HostClock::now();
                      const auto late = std::max(duration{}, std::chrono::duration_cast<duration>(now - deadline));
                      const auto missed = static_cast<std::uint64_t>(late / period);
                      {
                          std::scoped_lock lock(mutex_);
                          timing_ = {now - previous, late, missed};
                      }
                      target.step();
                      if constexpr (std::same_as<Mode, HostCadence>)
                          deadline += (missed + 1) * period;
                      else {
                          deadline += period;
                          const auto finished = HostClock::now();
                          if (deadline <= finished)
                              deadline = finished + period;
                      }
                      previous = now;
                  }
              } catch (...) {
                  std::scoped_lock lock(mutex_);
                  failure_ = std::current_exception();
              }
          }) {}
    ~PacedRunner() { stop(); }
    PacedRunner(const PacedRunner &) = delete;
    PacedRunner &operator=(const PacedRunner &) = delete;
    void stop() {
        thread_.request_stop();
        wake_.notify_all();
        if (thread_.joinable())
            thread_.join();
    }
    void rethrowFailure() const {
        std::scoped_lock lock(mutex_);
        if (failure_)
            std::rethrow_exception(failure_);
    }
    TimingSample timing() const {
        std::scoped_lock lock(mutex_);
        return timing_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable_any wake_;
    TimingSample timing_;
    std::exception_ptr failure_;
    std::jthread thread_;
};
} // namespace core
