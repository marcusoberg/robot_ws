#pragma once
#include "core/Block.hpp"
#include <utility>
namespace core {
template <SignalData T> class InputChannel;
template <SignalData T> class OutputChannel;
template <SignalData T> class InPort : public IInPort {
public:
    explicit InPort(Block &owner) { owner.registerInput(*this); }
    InPort(const InPort &) = delete;
    InPort &operator=(const InPort &) = delete;
    InPort(InPort &&) = delete;
    InPort &operator=(InPort &&) = delete;
    [[nodiscard]] const T &get() const noexcept { return *value_; }
    [[nodiscard]] IBlock *source() const noexcept override { return source_; }

private:
    template <SignalData> friend class OutPort;
    void bind(const T &value, IBlock &source) {
        requireConstruction();
        value_ = &value;
        source_ = &source;
    }
    const T *value_{};
    IBlock *source_{};
};
template <SignalData T> class OutPort {
public:
    OutPort(Block &owner, T initial) : owner_(owner), value_(std::move(initial)) {}
    OutPort(const OutPort &) = delete;
    OutPort &operator=(const OutPort &) = delete;
    OutPort(OutPort &&) = delete;
    OutPort &operator=(OutPort &&) = delete;
    void set(T value) { value_ = std::move(value); }
    [[nodiscard]] const T &get() const noexcept { return value_; }
    void operator>>(InPort<T> &input) { input.bind(value_, owner_); }
    void operator>>(InputChannel<T> &input) { *this >> input.port(); }
    void operator>>(OutputChannel<T> &output) { *this >> output.port(); }

private:
    Block &owner_;
    T value_;
};
template <SignalData T> class FeedbackPort final : public InPort<T> {
public:
    FeedbackPort(Block &owner, T initial) : InPort<T>(owner), initial_(std::move(initial)), previous_(initial_) {}
    [[nodiscard]] bool delayed() const noexcept override { return true; }
    [[nodiscard]] const T &get() const noexcept { return previous_; }
    void latch() override { previous_ = InPort<T>::get(); }
    void reset() override { previous_ = initial_; }

private:
    T initial_, previous_;
};
} // namespace core
