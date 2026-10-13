#pragma once
#include "core/mailbox.hpp"
#include "core/ports.hpp"
#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>
namespace core {
// Native channels participate in the DAG; asynchronous delivery uses Channel below.
template <SignalData T> class ChannelBlock final : public Block {
public:
    InPort<T> in{*this};
    OutPort<T> out;
    explicit ChannelBlock(T initial) : out(*this, std::move(initial)) {}
    void evaluate(const Context &) override { out.set(in.get()); }
};
template <SignalData T> class InputChannel {
public:
    explicit InputChannel(ChannelBlock<T> &block) : block_(block) {}
    InPort<T> &port() { return block_.in; }
    void operator>>(InPort<T> &input) { block_.out >> input; }

private:
    ChannelBlock<T> &block_;
};
template <SignalData T> class OutputChannel {
public:
    explicit OutputChannel(ChannelBlock<T> &block) : block_(block) {}
    InPort<T> &port() { return block_.in; }
    const T &read() const noexcept { return block_.out.get(); }
    void operator>>(InputChannel<T> &input) { block_.out >> input.port(); }
    void operator>>(InPort<T> &input) { block_.out >> input; }

private:
    ChannelBlock<T> &block_;
};
class Subscription {
public:
    Subscription() = default;
    explicit Subscription(std::function<void()> cancel) : cancel_(std::move(cancel)) {}
    Subscription(const Subscription &) = delete;
    Subscription &operator=(const Subscription &) = delete;
    Subscription(Subscription &&other) noexcept : cancel_(std::exchange(other.cancel_, {})) {}
    Subscription &operator=(Subscription &&other) noexcept {
        if (this != &other) {
            reset();
            cancel_ = std::exchange(other.cancel_, {});
        }
        return *this;
    }
    ~Subscription() { reset(); }
    void reset() {
        if (auto cancel = std::exchange(cancel_, {}))
            cancel();
    }

private:
    std::function<void()> cancel_;
};
// Latest-value fanout. Publication only exchanges storage; dispatch belongs to a worker.
// Callbacks receive an immutable snapshot, and subscription lifetimes are scoped.
template <SignalData T> class Channel {
    struct Slot {
        std::size_t id;
        std::function<void(const T &)> callback;
    };
    struct State {
        explicit State(T initial) : value(std::make_shared<const T>(std::move(initial))) {}
        std::mutex mutex;
        std::shared_ptr<const T> value;
        std::vector<Slot> subscribers;
        std::size_t revision{}, dispatched{}, next_id{};
    };

public:
    explicit Channel(T initial) : state_(std::make_shared<State>(std::move(initial))) {}
    void publish(T value) {
        auto immutable = std::make_shared<const T>(std::move(value));
        {
            std::scoped_lock lock(state_->mutex);
            immutable.swap(state_->value);
            ++state_->revision;
        }
    }
    [[nodiscard]] std::shared_ptr<const T> read() const {
        std::scoped_lock lock(state_->mutex);
        return state_->value;
    }
    Subscription subscribe(std::function<void(const T &)> callback) {
        std::scoped_lock lock(state_->mutex);
        const auto id = ++state_->next_id;
        state_->subscribers.push_back({id, std::move(callback)});
        return Subscription([weak = std::weak_ptr{state_}, id] {
            if (auto state = weak.lock()) {
                std::scoped_lock lock(state->mutex);
                std::erase_if(state->subscribers, [id](auto &s) { return s.id == id; });
            }
        });
    }
    // A single designated dispatcher owns callbacks. Shutdown quiesces it before captures die.
    std::size_t dispatch() {
        std::vector<Slot> slots;
        std::shared_ptr<const T> value;
        std::size_t coalesced;
        {
            std::scoped_lock lock(state_->mutex);
            if (state_->revision == state_->dispatched)
                return 0;
            coalesced = state_->revision - state_->dispatched - 1;
            state_->dispatched = state_->revision;
            value = state_->value;
            slots = state_->subscribers;
        }
        for (auto &slot : slots)
            slot.callback(*value);
        return coalesced;
    }

private:
    std::shared_ptr<State> state_;
};
} // namespace core
