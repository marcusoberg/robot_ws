#pragma once
#include <array>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

namespace core {
// Explicit delivery semantics: events cannot silently overwrite each other.
// Producers handle a full inbox at admission; consumers own all state transitions.
template <class T, std::size_t Capacity = 64>
    requires(Capacity > 0)
class Inbox {
public:
    [[nodiscard]] bool push(T value) {
        std::scoped_lock lock(mutex_);
        if (size_ == Capacity)
            return false;
        slots_[(head_ + size_) % Capacity].emplace(std::move(value));
        ++size_;
        return true;
    }
    [[nodiscard]] std::optional<T> pop() {
        std::scoped_lock lock(mutex_);
        if (size_ == 0)
            return std::nullopt;
        auto value = std::move(slots_[head_]);
        slots_[head_].reset();
        head_ = (head_ + 1) % Capacity;
        --size_;
        return value;
    }
    void clear() {
        std::scoped_lock lock(mutex_);
        for (auto &s : slots_)
            s.reset();
        head_ = size_ = 0;
    }

private:
    std::mutex mutex_;
    std::array<std::optional<T>, Capacity> slots_{};
    std::size_t head_{}, size_{};
};

// Copying an observation occurs after loading its shared immutable storage, outside the handoff.
template <class T> class Snapshot {
public:
    explicit Snapshot(T initial) : value_(std::make_shared<const T>(std::move(initial))) {}
    void publish(T value) {
        auto next = std::make_shared<const T>(std::move(value));
        {
            std::scoped_lock lock(mutex_);
            next.swap(value_);
        }
    }
    [[nodiscard]] T read() const {
        std::shared_ptr<const T> snapshot;
        {
            std::scoped_lock lock(mutex_);
            snapshot = value_;
        }
        return *snapshot;
    }
    [[nodiscard]] T exchange(T value) {
        auto next = std::make_shared<const T>(std::move(value));
        {
            std::scoped_lock lock(mutex_);
            next.swap(value_);
        }
        return *next;
    }

private:
    mutable std::mutex mutex_;
    std::shared_ptr<const T> value_;
};
} // namespace core
