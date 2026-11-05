#include "robot_runtime/parol6_serial.hpp"
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <mutex>
#include <numbers>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif

namespace robot_runtime {
namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
// Protocol and gearing from stock firmware revision 1e7fa639:
// main.cpp (command dispatch/packet layout), motor_init.cpp (ratios and standby).
// These decimal command bytes are not hexadecimal literals.
enum Command : int { Home = 100, Enable = 101, Disable = 102, TrackPosition = 156, Continue = 255 };
constexpr double motor_steps_per_turn = 200.;
constexpr double microsteps_per_step = 32.;
constexpr std::array<double, 6> ratios{6.4, 20., 20. * 38. / 42., 4., 4., 10.};
constexpr std::array<int, 6> home_counts{10240, -32000, 57905, 0, 0, 32000};
// Same bounds as the maintained PAROL6 URDF / upstream Commander.
constexpr std::array<double, 6> lower{-2.147573103039898,  -2.5308810043659533, 1.8826168509562036,
                                      -1.8407943987552895, -1.5707963267948966, 0.};
constexpr std::array<double, 6> upper{2.147573103039898,  -.05890486225480862,
                                      5.024235684484776,  1.8407943987552895,
                                      1.5707963267948966, 2. * std::numbers::pi};
constexpr double max_error = .06; // rad, time-aligned reference versus returned counter
double counts_per_rad(int i) {
    return motor_steps_per_turn * microsteps_per_step * ratios[i] / (2. * std::numbers::pi);
}
using Wire = std::array<unsigned char, 56>;
Wire packet(int command, const std::array<int, 6> &positions = {},
            const std::array<int, 6> &velocities = {}) {
    Wire b{};
    b[0] = b[1] = b[2] = 255;
    b[3] = 52;
    for (int i = 0; i < 6; ++i) {
        auto n = static_cast<std::uint32_t>(positions[i]);
        b[4 + 3 * i] = (n >> 16) & 255;
        b[5 + 3 * i] = (n >> 8) & 255;
        b[6 + 3 * i] = n & 255;
        auto v = static_cast<std::uint32_t>(velocities[i]);
        b[22 + 3 * i] = (v >> 16) & 255;
        b[23 + 3 * i] = (v >> 8) & 255;
        b[24 + 3 * i] = v & 255;
    }
    b[40] = command;
    b[41] = 255;
    b[53] = 212;
    b[54] = 1;
    b[55] = 2;
    // OUT1/OUT2 and all gripper fields remain zero.
    return b;
}
int signed24(const unsigned char *b) {
    int v = (int(b[0]) << 16) | (int(b[1]) << 8) | b[2];
    return (v & 0x800000) ? v - 0x1000000 : v;
}
int unsigned16(const unsigned char *b) { return (int(b[0]) << 8) | b[1]; }
std::runtime_error io_error(const char *operation) {
    return std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
void configure(int fd, int baud) {
    termios t{};
    if (tcgetattr(fd, &t))
        throw io_error("tcgetattr");
    cfmakeraw(&t);
    t.c_cflag &= ~(CRTSCTS | CSTOPB | PARENB | CSIZE);
    t.c_cflag |= CS8 | CLOCAL | CREAD;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
#ifdef __APPLE__
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
#else
    speed_t speed = baud == 3'000'000 ? B3000000 : B115200;
    cfsetispeed(&t, speed);
    cfsetospeed(&t, speed);
#endif
    if (tcsetattr(fd, TCSANOW, &t))
        throw io_error("tcsetattr");
#ifdef __APPLE__
    if (baud != 115200) {
        speed_t speed = baud;
        if (ioctl(fd, IOSSIOSPEED, &speed))
            throw io_error("IOSSIOSPEED");
    }
#endif
    // STM32 CDC needs DTR for replies. PTYs have no modem-control lines.
    int dtr = TIOCM_DTR, rts = TIOCM_RTS;
    if (ioctl(fd, TIOCMBIS, &dtr) && errno != ENOTTY)
        throw io_error("set DTR");
    if (ioctl(fd, TIOCMBIC, &rts) && errno != ENOTTY)
        throw io_error("clear RTS");
    if (tcflush(fd, TCIOFLUSH))
        throw io_error("tcflush");
}
void wait_fd(int fd, short events, Clock::time_point deadline) {
    while (true) {
        auto left =
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        if (left <= 0)
            throw std::runtime_error("serial response/write timeout");
        pollfd p{fd, events, 0};
        int r = poll(&p, 1, static_cast<int>(left));
        if (r < 0 && errno == EINTR)
            continue;
        if (r < 0)
            throw io_error("poll");
        if (r && (p.revents & (POLLERR | POLLHUP | POLLNVAL)))
            throw std::runtime_error("serial disconnected");
        if (r && (p.revents & events))
            return;
    }
}
void write_all(int fd, const Wire &b, Clock::time_point deadline) {
    std::size_t offset = 0;
    while (offset < b.size()) {
        wait_fd(fd, POLLOUT, deadline);
        auto n = ::write(fd, b.data() + offset, b.size() - offset);
        if (n < 0 && (errno == EAGAIN || errno == EINTR))
            continue;
        if (n <= 0)
            throw io_error("write");
        offset += n;
    }
}
struct Exchange {
    std::array<unsigned char, 60> reply{};
    Clock::time_point tx_begin{}, tx_end{}, rx_end{};
};
Exchange exchange_frame(int fd, const Wire &b) {
    Exchange exchange;
    auto deadline = Clock::now() + 100ms;
    exchange.tx_begin = Clock::now();
    write_all(fd, b, deadline);
    exchange.tx_end = Clock::now();
    auto &reply = exchange.reply;
    std::size_t offset = 0;
    while (offset < reply.size()) {
        wait_fd(fd, POLLIN, deadline);
        auto n = ::read(fd, reply.data() + offset, reply.size() - offset);
        if (n < 0 && (errno == EAGAIN || errno == EINTR))
            continue;
        if (n <= 0)
            throw std::runtime_error("serial EOF/read failure");
        offset += n;
    }
    exchange.rx_end = Clock::now();
    // Stock's 'CRC' is a constant, not a checksum. Do not claim error correction.
    if (reply[0] != 255 || reply[1] != 255 || reply[2] != 255 || reply[3] != 56 ||
        reply[57] != 212 || reply[58] != 1 || reply[59] != 2 || (reply[40] & 3) != 3 ||
        (reply[41] & 7) != 7 || (reply[42] & 3) != 3 || (reply[43] & 3) != 3 || reply[47] != b[40])
        throw std::runtime_error(
            "invalid stock response/command echo (wrong firmware or another controller)");
    return exchange;
}

std::int64_t ns(Clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}
using Joints = std::array<double, 6>;
struct Sample { Joints q{}, v{}; };
struct PathData { Positions times, joints; };
enum class MotionKind { Hold, Sampled, Move, Path, Live };
struct Motion {
    MotionKind kind{MotionKind::Hold};
    Clock::time_point epoch{};
    Joints initial{}, target{}, stage{}, velocity{};
    double duration{}, period{};
    std::shared_ptr<const PathData> path;

    Sample at(Clock::time_point time) const {
        Sample sample;
        double elapsed = std::max(0., std::chrono::duration<double>(time - epoch).count());
        if (kind == MotionKind::Move) {
            double u = std::clamp(elapsed / duration, 0., 1.);
            double s = std::clamp(u*u*u * (10. - 15.*u + 6.*u*u), 0., 1.);
            double rate = 30.*u*u*(1.-u)*(1.-u) / duration;
            for (int i = 0; i < 6; ++i) {
                sample.q[i] = std::lerp(initial[i], target[i], s);
                sample.v[i] = rate*(target[i]-initial[i]);
            }
        } else if (kind == MotionKind::Path) {
            if (elapsed >= duration) {
                std::copy_n(path->joints.end()-6, 6, sample.q.begin());
            } else {
                auto next = std::upper_bound(path->times.begin(), path->times.end(), elapsed);
                auto index = static_cast<std::size_t>(next - path->times.begin() - 1);
                double dt = path->times[index+1] - path->times[index];
                double u = (elapsed-path->times[index])/dt;
                for (int i = 0; i < 6; ++i) {
                    double a = path->joints[6*index+i], b = path->joints[6*(index+1)+i];
                    sample.q[i] = std::lerp(a, b, u);
                    sample.v[i] = (b-a)/dt;
                }
            }
        } else if (kind == MotionKind::Live) {
            // Exact same causal two-stage filter as JointStream, evaluated here
            // at serial dispatch. Retargeting retains both internal states.
            double u = elapsed/period, decay = std::exp(-u), weight = -std::expm1(-u);
            double ud = std::isfinite(u) ? u*decay : 0.;
            for (int i = 0; i < 6; ++i) {
                double low = std::min({initial[i], stage[i], target[i]});
                double high = std::max({initial[i], stage[i], target[i]});
                double intermediate = std::clamp(stage[i]+weight*(target[i]-stage[i]), low, high);
                sample.q[i] = std::clamp(initial[i]+ud*(stage[i]-initial[i])+
                                         std::max(0., weight-ud)*(target[i]-initial[i]), low, high);
                sample.v[i] = (intermediate-sample.q[i])/period;
            }
        } else {
            sample.q = target;
            sample.v = velocity;
        }
        return sample;
    }
    bool complete(Clock::time_point time) const {
        return (kind == MotionKind::Move || kind == MotionKind::Path) &&
               epoch != Clock::time_point{} &&
               std::chrono::duration<double>(time-epoch).count() >= duration;
    }
};
struct Dispatch {
    Wire wire{};
    Motion curve;
    Clock::time_point evaluated{}, applied_estimate{};
    std::uint64_t generation{}, motion_id{};
};
} // namespace

struct Parol6Serial::Impl {
    int fd{-1};
    mutable std::mutex mutex;
    std::jthread worker;
    Parol6Telemetry state;
    Clock::time_point received{}, lease{}, home_started{}, last_cycle{}, last_goal{};
    Motion motion;
    bool pending_start{}, pending_live{};
    Joints live_input{};
    Clock::time_point live_deadline{};
    std::uint64_t generation{};
    std::deque<Dispatch> history;
    std::deque<Parol6Telemetry> samples;
    bool saw_pressed{}, saw_unhomed{};
    bool position_invalidated{};
    int phase{}, stable{}, disabled_replies{};

    explicit Impl(const std::string &port, int baud) {
        if (baud != 3'000'000 && baud != 115200)
            throw std::invalid_argument("unsupported serial baud");
        fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0)
            throw io_error("open serial port");
        try {
            if (flock(fd, LOCK_EX | LOCK_NB) || ioctl(fd, TIOCEXCL))
                throw io_error("exclusive serial access");
            configure(fd, baud);
            worker = std::jthread([this](std::stop_token token) { run(token); });
        } catch (...) {
            ::close(fd);
            fd = -1;
            throw;
        }
    }
    ~Impl() { close(); }
    bool active() const {
        return state.mode == "Homing" || state.mode == "Enabling" || state.mode == "Running" || state.mode == "Ready" ||
               state.mode == "Stopping";
    }
    void fault(const std::string &reason, bool invalidate_position = true) {
        state.command_error.clear();
        if (state.mode != "Fault")
            state.reason = reason;
        state.mode = "Fault";
        position_invalidated |= invalidate_position;
        if (position_invalidated)
            state.homed_this_session = false;
    }
    bool reject(const std::string &reason) {
        state.command_error = reason;
        if (state.mode != "Fault")
            state.reason = reason;
        return false;
    }
    bool fresh() const {
        return fd >= 0 && state.frames && Clock::now() - received <= 100ms && !state.estop;
    }
    bool retained_home() const {
        if (position_invalidated || (state.homed_bits & 252) != 252 || state.position.size() != 6)
            return false;
        for (int i = 0; i < 6; ++i)
            if (!std::isfinite(state.position[i]) || state.position[i] < lower[i] || state.position[i] > upper[i])
                return false;
        return true;
    }
    bool recover(bool require_home) {
        state.command_error.clear();
        if (!fresh())
            return reject("fresh telemetry with E-stop released is required");
        if (state.mode == "Running" || state.mode == "Homing")
            return reject("pause the robot before reset or enable");
        if ((state.temperature_bits & 252) || (state.position_error_bits & 252) || state.timeout_value != 100)
            return reject("firmware error remains active");
        bool homed = retained_home();
        if (require_home && !homed)
            return reject("controller position is not valid; homing required");
        state.reason.clear();
        state.homed_this_session = homed;
        state.mode = homed ? "Enabling" : "Idle";
        phase = stable = 0;
        lease = Clock::now() + 250ms;
        return true;
    }
    bool input_ready() {
        state.command_error.clear();
        if (state.mode != "Running")
            return reject("motion input requires Running");
        auto now = Clock::now();
        if (now > lease) {
            fault("motion producer lease expired", false);
            return false;
        }
        if (last_goal != Clock::time_point{})
            state.goal_interval_seconds = std::chrono::duration<double>(now-last_goal).count();
        last_goal = now;
        return true;
    }
    bool valid_target(const Positions &q) {
        if (q.size() != 6) {
            fault("six joint angles in radians required", false);
            return false;
        }
        for (int i = 0; i < 6; ++i)
            if (!std::isfinite(q[i]) || q[i] < lower[i] || q[i] > upper[i]) {
                fault("J" + std::to_string(i+1) + " target outside joint bounds", false);
                return false;
            }
        return true;
    }
    bool can_plan(std::uint64_t id) {
        if (!id)
            return reject("trajectory requires a nonzero motion id");
        if (motion.kind != MotionKind::Hold)
            return reject("pause and activate before replacing a trajectory");
        return true;
    }
    void hold_motion() {
        motion = {};
        std::copy_n(state.position.begin(), 6, motion.target.begin());
        pending_start = pending_live = false;
        state.motion_id = 0;
        state.motion_elapsed_seconds = 0.;
        state.motion_complete = false;
        ++generation;
    }
    Sample sample_motion(Clock::time_point now) {
        if (pending_start) {
            motion.epoch = now;
            pending_start = false;
        }
        if (pending_live) {
            auto previous = motion.at(now);
            bool was_live = motion.kind == MotionKind::Live;
            motion.initial = previous.q;
            for (int i = 0; i < 6; ++i)
                motion.stage[i] = was_live ? previous.q[i] + previous.v[i]*motion.period : previous.q[i];
            motion.target = live_input;
            motion.kind = MotionKind::Live;
            motion.epoch = now;
            pending_live = false;
        }
        state.motion_elapsed_seconds = motion.epoch == Clock::time_point{} ? 0. :
            std::max(0., std::chrono::duration<double>(now-motion.epoch).count());
        state.motion_complete = motion.complete(now);
        return motion.at(now);
    }
    Wire command(Clock::time_point now) {
        if (last_cycle != Clock::time_point{})
            state.cycle_seconds = std::chrono::duration<double>(now - last_cycle).count();
        if (active() && last_cycle != Clock::time_point{} && now - last_cycle > 100ms)
            fault("native polling deadline expired");
        if (active() && now > lease)
            fault("producer lease expired", state.mode == "Homing");
        last_cycle = now;
        if (state.mode == "Enabling")
            return packet(phase < 2 ? Enable : Continue);
        if (state.mode == "Homing") {
            if (now - home_started > 90s) {
                fault("homing timed out");
                return packet(Disable);
            }
            // 101 must be dispatched before 100. 100 is sent ONCE; 255 continues
            // homing while keeping E-stop feedback flowing. 255 does NOT stop it.
            return packet(phase < 2 ? Enable : phase == 2 ? Home : Continue);
        }
        // Commander dummy_data: zero velocities with 255 holds the enabled arm.
        // Sending 102 here would release holding torque and invalidate its pose.
        if (state.mode == "Ready" || state.mode == "Stopping") {
            std::array<int, 6> positions{};
            std::copy(state.steps.begin(), state.steps.end(), positions.begin());
            return packet(Continue, positions);
        }
        if (state.mode == "Running") {
            if (motion.kind == MotionKind::Live && now > live_deadline) {
                fault("live target feedback expired", false);
                return packet(Disable);
            }
            auto sample = sample_motion(now);
            std::array<int, 6> target{};
            std::array<int, 6> velocity{};
            for (int i = 0; i < 6; ++i) {
                double wire_speed = sample.v[i]*counts_per_rad(i);
                if (!std::isfinite(sample.q[i]) || sample.q[i] < lower[i] || sample.q[i] > upper[i] ||
                    !std::isfinite(wire_speed) || wire_speed < -8388608. || wire_speed > 8388607.) {
                    fault("trajectory sample outside joint bounds or signed 24-bit velocity range", false);
                    return packet(Disable);
                }
                int desired = static_cast<int>(std::llround(sample.q[i] * counts_per_rad(i)));
                target[i] =
                    std::clamp(desired, static_cast<int>(std::ceil(lower[i] * counts_per_rad(i))),
                               static_cast<int>(std::floor(upper[i] * counts_per_rad(i))));
                velocity[i] = static_cast<int>(std::llround(wire_speed));
            }
            return packet(TrackPosition, target, velocity);
        }
        return packet(Disable);
    }
    void decode_telemetry(const std::array<unsigned char, 60> &b) {
        state.steps.resize(6);
        state.speed_steps.resize(6);
        state.position.resize(6);
        state.velocity.resize(6);
        for (int i = 0; i < 6; ++i) {
            state.steps[i] = signed24(&b[4 + 3 * i]);
            state.speed_steps[i] = signed24(&b[22 + 3 * i]);
            state.position[i] = state.steps[i] / counts_per_rad(i);
            state.velocity[i] = state.speed_steps[i] / counts_per_rad(i);
        }
        state.payload.assign(b.begin() + 4, b.end());
        state.homed_bits = b[40];
        state.io_bits = b[41];
        state.estop = !(b[41] & 8);
        state.temperature_bits = b[42];
        state.position_error_bits = b[43];
        state.timing_ticks = unsigned16(&b[44]);
        state.timeout_value = b[46];
        state.command_echo = b[47];
        state.gripper_id = b[48];
        state.gripper_position = unsigned16(&b[49]);
        state.gripper_speed = unsigned16(&b[51]);
        state.gripper_current = unsigned16(&b[53]);
        state.gripper_status = b[55];
        state.object_detection = b[56];
    }
    void update_motion_state(int sent, Clock::time_point now, const std::vector<int> &old_steps) {
        if (state.mode == "Enabling") {
            if (sent == Enable)
                ++phase;
            stable = sent == Continue && old_steps == state.steps ? stable + 1 : 0;
            if (stable >= 3)
                state.mode = "Ready";
        } else if (state.mode == "Homing") {
            if ((phase < 2 && sent == Enable) || (phase == 2 && sent == Home))
                ++phase;
            if (sent == Continue) {
                saw_unhomed |= (state.homed_bits & 252) != 252;
                bool at_home = saw_unhomed && (state.homed_bits & 252) == 252;
                for (int i = 0; i < 6; ++i)
                    at_home &= std::abs(state.steps[i] - home_counts[i]) <= 2;
                stable = at_home ? stable + 1 : 0;
                if (stable >= 10 && now - home_started > 1s) {
                    position_invalidated = false;
                    state.homed_this_session = true;
                    state.mode = "Ready";
                    stable = 0;
                }
            }
        } else if (state.mode == "Stopping") {
            stable = old_steps == state.steps && sent == Continue ? stable + 1 : 0;
            if (stable >= 3)
                state.mode = "Ready";
        }
    }
    void accept(const Exchange &exchange, Dispatch dispatched) {
        const auto &b = exchange.reply;
        int sent = dispatched.wire[40];
        auto now = exchange.rx_end;
        auto old_steps = state.steps;
        received = now;
        ++state.frames;
        state.connected = true;
        decode_telemetry(b);
        // Firmware snapshots counters BEFORE Get_data unpacks this packet, and
        // applies its command only on the next firmware iteration. Its echo is
        // receipt confirmation, not an applied-command/sample timestamp.
        // The midpoint estimates packet arrival. RTT/2 covers host transport
        // ambiguity only; firmware snapshot/dispatch latency is unobservable.
        auto sample_time = exchange.tx_begin + (exchange.rx_end-exchange.tx_begin)/2;
        state.sample_time_ns = ns(sample_time);
        state.receive_time_ns = ns(now);
        state.transmit_time_ns = ns(exchange.tx_begin);
        state.write_complete_time_ns = ns(exchange.tx_end);
        state.evaluation_time_ns = ns(dispatched.evaluated);
        state.reference_time_ns = 0;
        state.reference_frame = 0;
        state.reference_motion_id = 0;
        state.transmitted_motion_id = dispatched.motion_id;
        state.sample_wall_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() +
            ns(sample_time) - ns(Clock::now());
        state.round_trip_seconds = std::chrono::duration<double>(now-exchange.tx_begin).count();
        state.timing_uncertainty_seconds = state.round_trip_seconds/2.;
        state.transmitted_position.resize(6);
        state.transmitted_velocity.resize(6);
        for (int i = 0; i < 6; ++i) {
            state.transmitted_position[i] = signed24(&dispatched.wire[4+3*i])/counts_per_rad(i);
            state.transmitted_velocity[i] = signed24(&dispatched.wire[22+3*i])/counts_per_rad(i);
        }
        state.reference = state.position;
        state.reference_valid = false;
        if (!history.empty()) {
            const auto &previous = history.back();
            if (previous.wire[40] == TrackPosition && previous.generation == generation) {
                // Continue the last *transmitted* curve over its execution
                // interval. Never use a newly queued target or a stale waypoint.
                auto reference_time = previous.evaluated + std::max(Clock::duration::zero(),
                                                    sample_time-previous.applied_estimate);
                auto reference = previous.curve.at(reference_time);
                state.reference.assign(reference.q.begin(), reference.q.end());
                state.reference_valid = true;
                state.reference_frame = state.frames-1;
                state.reference_motion_id = previous.motion_id;
                state.reference_time_ns = ns(reference_time);
            }
        }
        dispatched.applied_estimate = sample_time;
        history.push_back(std::move(dispatched));
        if (history.size() > 256)
            history.pop_front();
        if (sent == Disable)
            ++disabled_replies;
        else
            disabled_replies = 0;
        if (state.mode == "Connecting" && disabled_replies >= 3) {
            state.mode = "Idle";
            state.homed_this_session = retained_home();
        }
        if (state.mode == "Idle" && disabled_replies >= 3 && state.estop)
            saw_pressed = true;
        state.estop_verified = saw_pressed && !state.estop;
        // An input cycle while already disabled is not a new motion fault.
        // Retained homing alone does not mean the drivers are enabled.
        // fresh() still blocks home/enable/start until the input is released.
        if (state.estop && active())
            fault("E-stop input asserted during enabled operation");
        if ((state.temperature_bits & 252) || (state.position_error_bits & 252) ||
            state.timeout_value != 100)
            fault("firmware error flags or unsupported timeout field");
        update_motion_state(sent, now, old_steps);
        if (state.homed_this_session) {
            if ((state.homed_bits & 252) != 252)
                fault("homing flags lost; possible controller reset");
            for (int i = 0; i < 6; ++i) {
                if (state.position[i] < lower[i] || state.position[i] > upper[i])
                    fault("counter outside joint limits");
            }
        }
        if (sent == Disable && disabled_replies >= 3 && old_steps.size() == 6 &&
            old_steps != state.steps)
            fault("step counters changed while disabled");
        if (state.mode == "Running" && state.reference_valid)
            for (int i = 0; i < 6; ++i)
                if (std::abs(state.reference[i]-state.position[i]) > max_error) {
                    fault("J" + std::to_string(i+1) + " tracking error " +
                          std::to_string(state.reference[i]-state.position[i]) +
                          " rad exceeds " + std::to_string(max_error) +
                          " rad (time-aligned reference)", false);
                    break;
                }
        samples.push_back(state);
        if (samples.size() > 512)
            samples.pop_front();
    }
    void run(std::stop_token token) noexcept {
        auto next_cycle = Clock::now();
        while (!token.stop_requested()) {
                try {
                Dispatch dispatched;
                {
                    std::lock_guard lock(mutex);
                    dispatched.evaluated = Clock::now();
                    dispatched.wire = command(dispatched.evaluated);
                    dispatched.curve = motion;
                    dispatched.generation = generation;
                    dispatched.motion_id = state.motion_id;
                }
                auto exchange = exchange_frame(fd, dispatched.wire);
                {
                    std::lock_guard lock(mutex);
                    accept(exchange, std::move(dispatched));
                }
            } catch (const std::exception &e) {
                {
                    std::lock_guard lock(mutex);
                    state.connected = false;
                    fault(e.what());
                }
                // Write disable even when replies are lost/malformed. No motion
                // packet is ever retried after a communication failure.
                try {
                    write_all(fd, packet(Disable), Clock::now() + 20ms);
                } catch (...) {
                }
            } catch (...) {
                std::lock_guard lock(mutex);
                state.connected = false;
                fault("native serial worker failed");
            }
            // Keep a fixed 100 Hz clock instead of accumulating each wake-up's
            // lateness. Never burst multiple packets to catch up after an overrun.
            next_cycle += 10ms;
            auto finished = Clock::now();
            if (next_cycle <= finished)
                next_cycle += (std::chrono::duration_cast<std::chrono::milliseconds>(
                                   finished - next_cycle).count() / 10 + 1) * 10ms;
            std::this_thread::sleep_until(next_cycle);
        }
        // An echoed 102 precedes firmware dispatch; repeat it across loop cycles.
        for (int i = 0; i < 3; ++i) {
            try {
                write_all(fd, packet(Disable), Clock::now() + 20ms);
            } catch (...) {
            }
            std::this_thread::sleep_for(10ms);
        }
    }
    void close() {
        if (worker.joinable()) {
            worker.request_stop();
            worker.join();
        }
        std::lock_guard lock(mutex);
        if (fd >= 0) {
            ioctl(fd, TIOCNXCL);
            ::close(fd);
            fd = -1;
        }
        state.connected = false;
        state.homed_this_session = false;
        state.mode = "Closed";
    }
};

Parol6Serial::Parol6Serial(const std::string &port, int baud)
    : impl_(std::make_unique<Impl>(port, baud)) {}
Parol6Serial::~Parol6Serial() = default;
Positions Parol6Serial::standby() {
    Positions q(6);
    for (int i = 0; i < 6; ++i)
        q[i] = home_counts[i] / counts_per_rad(i);
    return q;
}
Parol6Telemetry Parol6Serial::observe() const {
    std::lock_guard lock(impl_->mutex);
    auto s = impl_->state;
    s.age_seconds =
        s.frames ? std::chrono::duration<double>(Clock::now() - impl_->received).count() : -1.;
    s.connected &= s.age_seconds >= 0 && s.age_seconds < .1;
    return s;
}
std::vector<Parol6Telemetry> Parol6Serial::drain_samples() {
    std::lock_guard lock(impl_->mutex);
    std::vector<Parol6Telemetry> result(impl_->samples.begin(), impl_->samples.end());
    impl_->samples.clear();
    return result;
}
bool Parol6Serial::home() {
    std::lock_guard lock(impl_->mutex);
    impl_->state.command_error.clear();
    if (!impl_->fresh())
        return impl_->reject("fresh telemetry with E-stop released is required");
    if (impl_->state.mode != "Idle" || !impl_->state.estop_verified)
        return impl_->reject(
            "home requires Idle and an observed E-stop press/release on this connection");
    impl_->state.reason.clear();
    impl_->state.mode = "Homing";
    impl_->position_invalidated = true;
    impl_->state.homed_this_session = false;
    impl_->phase = impl_->stable = 0;
    impl_->saw_unhomed = false;
    impl_->home_started = Clock::now();
    impl_->lease = Clock::now() + 250ms;
    return true;
}
void Parol6Serial::keepalive() {
    std::lock_guard lock(impl_->mutex);
    if (impl_->state.mode == "Homing" || impl_->state.mode == "Enabling" || impl_->state.mode == "Ready" ||
        impl_->state.mode == "Stopping") {
        if (Clock::now() > impl_->lease) {
            impl_->fault("server heartbeat expired", impl_->state.mode == "Homing");
            return;
        }
        impl_->lease = Clock::now() + 250ms;
    }
}
bool Parol6Serial::enable() {
    std::lock_guard lock(impl_->mutex);
    return impl_->recover(true);
}
bool Parol6Serial::reset() {
    std::lock_guard lock(impl_->mutex);
    return impl_->recover(false);
}
bool Parol6Serial::start() {
    std::lock_guard lock(impl_->mutex);
    impl_->state.command_error.clear();
    if (!impl_->fresh())
        return impl_->reject("fresh telemetry with E-stop released is required");
    if (impl_->state.mode != "Ready" || !impl_->state.homed_this_session)
        return impl_->reject("start requires completed homing and Ready state");
    impl_->hold_motion();
    impl_->history.clear();
    impl_->state.reason.clear();
    impl_->state.mode = "Running";
    impl_->phase = 0;
    impl_->lease = Clock::now() + 250ms;
    return true;
}
bool Parol6Serial::submit(const Positions &q, const Positions &velocities) {
    std::lock_guard lock(impl_->mutex);
    if (!impl_->input_ready() || !impl_->valid_target(q))
        return false;
    if (impl_->motion.kind != MotionKind::Hold && impl_->motion.kind != MotionKind::Sampled)
        return impl_->reject("sampled goals cannot replace an executing trajectory");
    if (!velocities.empty() && velocities.size() != 6) {
        impl_->fault("six feed-forward velocities in rad/s required", false);
        return false;
    }
    for (int i = 0; i < 6; ++i) {
        // Legacy immediate targets retain the old jump guard, but compare
        // command continuity, not a future command against an old measurement.
        if (std::abs(q[i]-impl_->motion.target[i]) > max_error) {
            impl_->fault("sampled command discontinuity exceeds " + std::to_string(max_error) +
                         " rad; submit a timed move", false);
            return false;
        }
        double v = velocities.empty() ? 0. : velocities[i];
        if (!std::isfinite(v) || v*counts_per_rad(i) < -8388608. || v*counts_per_rad(i) > 8388607.) {
            impl_->fault("feed-forward velocity outside signed 24-bit wire range", false);
            return false;
        }
    }
    impl_->motion.kind = MotionKind::Sampled;
    std::copy_n(q.begin(), 6, impl_->motion.target.begin());
    for (int i = 0; i < 6; ++i)
        impl_->motion.velocity[i] = velocities.empty() ? 0. : velocities[i];
    impl_->lease = Clock::now() + 250ms;
    return true;
}
bool Parol6Serial::move_joint(const Positions &q, double duration, std::uint64_t id) {
    std::lock_guard lock(impl_->mutex);
    if (!impl_->input_ready() || !impl_->valid_target(q) || !impl_->can_plan(id))
        return false;
    if (!std::isfinite(duration) || duration <= 0.)
        return impl_->reject("move duration must be finite and positive");
    for (int i = 0; i < 6; ++i) {
        // Quintic peak derivative is 1.875/duration. Validate wire capacity
        // before starting, including moves shorter than one serial interval.
        double peak = 1.875*(q[i]-impl_->motion.target[i])/duration*counts_per_rad(i);
        if (!std::isfinite(peak) || peak < -8388608. || peak > 8388607.)
            return impl_->reject("move velocity outside signed 24-bit wire range");
    }
    impl_->motion.initial = impl_->motion.target;
    impl_->motion.kind = MotionKind::Move;
    impl_->motion.duration = duration;
    std::copy_n(q.begin(), 6, impl_->motion.target.begin());
    impl_->pending_start = true;
    impl_->state.motion_id = id;
    impl_->state.motion_complete = false;
    impl_->lease = Clock::now() + 250ms;
    return true;
}
bool Parol6Serial::follow_path(const Positions &times, const Positions &q, std::uint64_t id) {
    // Build immutable storage outside the serial mutex. No IK/planning in IO.
    auto path = std::make_shared<PathData>(PathData{times, q});
    std::lock_guard lock(impl_->mutex);
    if (!impl_->input_ready() || !impl_->can_plan(id))
        return false;
    if (times.size() < 2 || q.size() != times.size()*6 || times.front() != 0.)
        return impl_->reject("path requires times starting at zero and six positions per knot");
    for (std::size_t n = 0; n < times.size(); ++n) {
        if (!std::isfinite(times[n]) || (n && times[n] <= times[n-1]))
            return impl_->reject("path times must be finite and strictly increasing");
        for (int i = 0; i < 6; ++i) {
            if (!std::isfinite(q[6*n+i]) || q[6*n+i] < lower[i] || q[6*n+i] > upper[i])
                return impl_->reject("path knot outside joint bounds");
            if (n) {
                double speed = (q[6*n+i]-q[6*(n-1)+i])/(times[n]-times[n-1])*counts_per_rad(i);
                if (!std::isfinite(speed) || speed < -8388608. || speed > 8388607.)
                    return impl_->reject("path velocity outside signed 24-bit wire range");
            } else if (std::abs(q[i]-impl_->motion.target[i]) > .001) {
                return impl_->reject("path start differs from held position; replan from current feedback");
            }
        }
    }
    impl_->motion.kind = MotionKind::Path;
    impl_->motion.duration = times.back();
    impl_->motion.path = std::move(path);
    impl_->pending_start = true;
    impl_->state.motion_id = id;
    impl_->state.motion_complete = false;
    impl_->lease = Clock::now() + 250ms;
    return true;
}
bool Parol6Serial::live_target(const Positions &q, double period, std::uint64_t id) {
    std::lock_guard lock(impl_->mutex);
    if (!impl_->input_ready() || !impl_->valid_target(q))
        return false;
    if (!id || !std::isfinite(period) || period <= 0.)
        return impl_->reject("live target requires a motion id and positive smoothing period");
    if (impl_->motion.kind != MotionKind::Live && impl_->motion.kind != MotionKind::Hold &&
        (impl_->pending_start || !impl_->motion.complete(Clock::now())))
        return impl_->reject("finish the initial move before starting live tracking");
    if (impl_->motion.kind == MotionKind::Live &&
        (id != impl_->state.motion_id || period != impl_->motion.period))
        return impl_->reject("live motion id and smoothing period must remain unchanged");
    if (impl_->motion.kind == MotionKind::Live && Clock::now() > impl_->live_deadline) {
        impl_->fault("live target feedback expired", false);
        return false;
    }
    std::copy_n(q.begin(), 6, impl_->live_input.begin());
    impl_->motion.period = period;
    impl_->pending_live = true;
    impl_->state.motion_id = id;
    impl_->state.motion_complete = false;
    impl_->live_deadline = Clock::now() + 300ms;
    impl_->lease = Clock::now() + 250ms;
    return true;
}
bool Parol6Serial::heartbeat(std::uint64_t id) {
    std::lock_guard lock(impl_->mutex);
    if (!impl_->input_ready())
        return false;
    if (!id || id != impl_->state.motion_id)
        return impl_->reject("heartbeat belongs to a different motion");
    impl_->lease = Clock::now() + 250ms;
    return true;
}
void Parol6Serial::pause() {
    std::lock_guard lock(impl_->mutex);
    if (impl_->state.mode == "Running") {
        impl_->state.mode = "Stopping";
        impl_->stable = 0;
        impl_->lease = Clock::now() + 250ms;
    } else if (impl_->state.mode == "Homing") {
        // 255 continues homing: only 102 can abort it in stock firmware.
        impl_->state.mode = "Stopped";
        impl_->state.homed_this_session = false;
    }
}
void Parol6Serial::stop() {
    std::lock_guard lock(impl_->mutex);
    if (impl_->state.mode != "Fault" && impl_->state.mode != "Closed")
        impl_->state.mode = "Stopped";
    impl_->state.homed_this_session = false;
}
void Parol6Serial::close() { impl_->close(); }
} // namespace robot_runtime
