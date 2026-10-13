#pragma once
#include "application/compose.hpp"
#include "application/source.hpp"
#include "application/capture.hpp"
#include <map>
#include <atomic>
#include <mutex>
#include <limits>
#include <condition_variable>
#include <set>
#include <cmath>
#include <thread>
#include "application/sensor_inbox.hpp"

namespace application {
// The application owns lifetime and scheduling. Neither Python nor a transport
// receives a mutable Execution, graph, backend, or driver reference.
class Runtime final {
    struct Owner {
        virtual ~Owner() = default;
        virtual robot::Execution &execution() = 0;
        virtual void advance(std::size_t) = 0;
        virtual void reset() = 0;
        virtual void close() = 0;
        virtual void failure() const = 0;
    };
    template <class Device, bool = robot::backend::Stepped<Device>> struct DriverType {
        using type = robot::execution::drivers::Host;
    };
    template <class Device> struct DriverType<Device, true> {
        using type = robot::execution::drivers::Simulation<Device>;
    };
    template <robot::backend::Device Device> class DeviceOwner final : public Owner {
        using Driver = typename DriverType<Device>::type;
        Device *device_; // Borrowed from the address-stable Backend owned by execution_.
        robot::Execution execution_;
        Driver driver_;
        std::unique_ptr<core::PacedRunner<Driver, typename Driver::Pacing>> runner_;
        mutable std::mutex execution_mutex_;
        std::atomic_bool closing_{};
        static Driver makeDriver(robot::Execution &execution, Device &device, const Deployment &deployment) {
            if constexpr (robot::backend::Stepped<Device>)
                return Driver{execution, device, deployment.control_period};
            else
                return Driver{execution, deployment.control_period, deployment.host_timing.value_or(
                    robot::execution::drivers::HostTiming{core::Period{2 * deployment.control_period.value()}})};
        }
    public:
        DeviceOwner(std::unique_ptr<Device> device, const Deployment &deployment, RunMode mode)
            : device_(device.get()), execution_(robot::Backend{std::move(device)}, deployment.controller, deployment.control_period,
                                               deployment.standby, deployment.standby_duration),
              driver_(makeDriver(execution_, *device_, deployment)) {
            if constexpr (robot::backend::Stepped<Device>)
                driver_.reset(); // Publish initial feedback without advancing physics.
            else {
                if (mode == RunMode::Stepped) throw std::invalid_argument("hardware requires realtime execution");
                driver_.step();
            }
            if (mode == RunMode::Realtime)
                runner_ = std::make_unique<core::PacedRunner<Driver, typename Driver::Pacing>>(driver_);
        }
        ~DeviceOwner() override { close(); }
        robot::Execution &execution() override { return execution_; }
        void advance(std::size_t ticks) override {
            std::scoped_lock lock(execution_mutex_);
            if (runner_) throw std::invalid_argument("step requires stepped execution");
            if constexpr (robot::backend::Stepped<Device>) {
                for (std::size_t i = 0; i < ticks; ++i) {
                    if (closing_.load()) throw std::runtime_error("step interrupted by runtime closure");
                    driver_.step();
                }
            }
            else throw std::invalid_argument("hardware cannot be explicitly stepped");
        }
        void reset() override {
            std::scoped_lock lock(execution_mutex_);
            if constexpr (requires { driver_.reset(); }) {
                const bool paced = bool(runner_);
                if (paced) { runner_->stop(); runner_.reset(); }
                driver_.reset();
                if (paced) runner_ = std::make_unique<core::PacedRunner<Driver, typename Driver::Pacing>>(driver_);
            } else throw std::invalid_argument("reset requires a resettable simulation");
        }
        void close() override {
            closing_.store(true);
            std::scoped_lock lock(execution_mutex_);
            if (!runner_) return;
            const auto channel = execution_.communication();
            const auto generation = channel->latest().status.generation;
            (void)channel->request({robot::communication::OperatorAction::Stop});
            const auto deadline = core::HostClock::now() + std::chrono::seconds(5);
            while (core::HostClock::now() < deadline) {
                const auto status = channel->latest().status;
                if (status.mode == robot::execution::Mode::Fault ||
                    (status.generation > generation && (status.mode == robot::execution::Mode::Hold || status.mode == robot::execution::Mode::Disarmed))) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            runner_->stop(); runner_.reset();
        }
        void failure() const override { std::scoped_lock lock(execution_mutex_); if (runner_) runner_->rethrowFailure(); }
    };
public:
    Runtime(const DeploymentSpecification &specification, const std::string &backend, RunMode mode, std::optional<double> action_period = {})
        : mode_(mode) {
        auto deployment = loadDeployment(specification);
        sensors_ = std::make_shared<SensorInbox>(specification.external_sensors);
        control_period_ = deployment.control_period.value();
        const auto period = action_period ? seconds(*action_period) : control_period_;
        if (period % control_period_ != core::duration{}) throw std::invalid_argument("action period must contain whole control intervals");
        action_ticks_ = period / control_period_;
        withBackend(backend, deployment, [&](auto device) {
            if constexpr (requires { device->sensorDescriptions(); })
                for (const auto &description : specification.external_sensors)
                    if (std::ranges::find(device->sensorDescriptions(), description.name, &robot::resources::SensorDescription::name)
                        != device->sensorDescriptions().end()) throw std::invalid_argument("external sensor collides with a backend sensor");
            using Device = robot::backend::WithSensors<typename decltype(device)::element_type, SensorProvider>;
            auto bound = std::make_unique<Device>(std::move(device), std::make_unique<SensorProvider>(sensors_));
            owner_ = std::make_shared<DeviceOwner<Device>>(std::move(bound), deployment, mode);
            return 0;
        });
        channel_ = owner_->execution().communication();
        instance_ = std::to_string(++next_runtime_) + "-" + std::to_string(core::HostClock::now().time_since_epoch().count());
        operator_source_=std::make_shared<Source>(channel_,alive_,robot::communication::SourceId{++next_source_},mode_,instance_,"operator",instance_);
    }
    ~Runtime() { close(); }
    Runtime(const Runtime &) = delete;
    Runtime &operator=(const Runtime &) = delete;
    const std::shared_ptr<robot::Communication> &communication() const { return channel_; }
    const std::string &instance() const { return instance_; }
    std::shared_ptr<Source> source(const std::string &name,const std::string &instance) {
        (void)owner();
        if(name.empty() || instance.empty() || name.size()>256 || instance.size()>256) throw std::invalid_argument("source name and instance must contain 1..256 bytes");
        std::scoped_lock lock(sources_mutex_); const auto key=std::pair{name,instance};
        if(const auto found=sources_.find(key);found!=sources_.end()) return found->second;
        auto value=std::make_shared<Source>(channel_,alive_,robot::communication::SourceId{++next_source_},mode_,instance_,name,instance);
        if(sources_.size()>=64) throw std::invalid_argument("runtime source capacity exceeded");
        sources_.emplace(key,value); return value;
    }
    const std::shared_ptr<Source> &operatorSource() const { return operator_source_; }
    robot::execution::Snapshot snapshot() const { owner()->failure(); auto value=channel_->latest(); value.runtime_instance=instance_; return value; }
    std::pair<std::uint64_t,bool> operate(robot::communication::OperatorAction action) { return operator_source_->operate(action); }
    void step(std::size_t count) {
        if (mode_ != RunMode::Stepped) throw std::invalid_argument("step requires stepped execution");
        if (count > std::numeric_limits<std::size_t>::max() / action_ticks_) throw std::invalid_argument("step count exceeds clock range");
        owner()->advance(count * action_ticks_);
    }
    void tick() { owner()->advance(1); }
    void reset() { sensors_->clear(); std::scoped_lock lock(capture_mutex_); owner()->reset(); capture_.reset(); }
    void publishSensor(robot::backend::SensorSample sample) {
        if (sample.stamp.epoch != snapshot().epoch) throw std::invalid_argument("sensor sample belongs to a previous timeline");
        sensors_->publish(std::move(sample));
    }
    void waitChange(std::uint64_t epoch, std::uint64_t sequence, double timeout) const {
        const auto until = core::HostClock::now() + seconds(timeout);
        do {
            const auto value = snapshot();
            if (value.epoch != epoch || value.observation_sequence != sequence) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        } while (core::HostClock::now() < until);
    }
    void close() {
        if (closed_.exchange(true)) return;
        alive_->store(false);
        std::shared_ptr<Owner> retiring;
        { std::scoped_lock lock(owner_mutex_); retiring = std::exchange(owner_, {}); }
        if (retiring) retiring->close();
    }
private:
    RunMode mode_;
    std::shared_ptr<Owner> owner_;
    mutable std::mutex owner_mutex_;
    std::shared_ptr<Owner> owner() const {
        std::scoped_lock lock(owner_mutex_);
        if (!owner_) throw std::logic_error("runtime is closed");
        return owner_;
    }
    std::shared_ptr<robot::Communication> channel_;
    std::shared_ptr<SensorInbox> sensors_;
    core::duration control_period_{};
    std::size_t action_ticks_{};
    inline static std::atomic_uint64_t next_source_{},next_runtime_{};
    std::atomic_bool closed_{};
    std::shared_ptr<std::atomic_bool> alive_=std::make_shared<std::atomic_bool>(true);
    std::string instance_;
    std::shared_ptr<Source> operator_source_;
    std::mutex sources_mutex_,capture_mutex_;
    std::map<std::pair<std::string,std::string>,std::shared_ptr<Source>> sources_;
    std::shared_ptr<const Capture> capture_;
    std::uint64_t capture_sequence_{};
public:
    void publishCapture(const robot::execution::Snapshot &basis,std::vector<ImageSample> images) {
        if(basis.runtime_instance!=instance_ || basis.epoch!=snapshot().epoch) throw std::invalid_argument("capture belongs to a previous timeline");
        if(images.empty()) throw std::invalid_argument("capture needs image samples");
        std::size_t total=0; std::set<std::string> names;
        for(const auto &sample:images) {
            if(sample.name.empty() || !names.insert(sample.name).second || sample.stamp.epoch!=basis.epoch || sample.stamp.source_clock.empty())
                throw std::invalid_argument("invalid capture identity or source clock");
            const auto &image=sample.image; const auto channels=image.encoding=="rgb8"?3:image.encoding=="32FC1"?4:0;
            if(!channels || !image.width || !image.height || image.stride!=std::uint64_t(image.width)*channels || !image.bytes ||
               image.bytes->size()!=std::uint64_t(image.stride)*image.height || image.bytes->size()>256ULL*1024*1024-total)
                throw std::invalid_argument("capture pixels disagree with encoding or exceed memory budget");
            total+=image.bytes->size();
            for(double x:sample.intrinsics) if(!std::isfinite(x)) throw std::invalid_argument("nonfinite camera intrinsics");
            for(double x:sample.world_from_camera) if(!std::isfinite(x)) throw std::invalid_argument("nonfinite camera extrinsics");
        }
        std::scoped_lock lock(capture_mutex_);
        if(basis.epoch!=snapshot().epoch) throw std::invalid_argument("capture belongs to a previous timeline");
        capture_=std::make_shared<const Capture>(Capture{instance_,std::make_shared<const robot::execution::Snapshot>(basis),std::move(images),++capture_sequence_});
    }
    std::shared_ptr<const Capture> capture() {
        (void)owner(); std::scoped_lock lock(capture_mutex_); return capture_;
    }
};
} // namespace application
