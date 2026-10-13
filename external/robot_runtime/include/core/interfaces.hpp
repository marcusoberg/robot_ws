#pragma once
#include "core/types.hpp"
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace core {
class IInPort {
public:
    virtual ~IInPort() = default;
    [[nodiscard]] virtual IBlock *source() const noexcept = 0;
    [[nodiscard]] virtual bool delayed() const noexcept { return false; }
    virtual void latch() {}
    virtual void reset() {}

private:
    friend class Graph;
    bool prepared_{};
    void prepare() noexcept { prepared_ = true; }

protected:
    void requireConstruction() const {
        if (prepared_)
            throw std::logic_error("graph wiring is fixed after preparation");
    }
};
class IBlock {
public:
    virtual ~IBlock() = default;
    virtual void evaluate(const Context &) = 0;
    virtual void reset() = 0;
    [[nodiscard]] virtual std::span<IInPort *const> inputs() const noexcept = 0;
};
class IComponent {
public:
    IComponent() = default;
    IComponent(const IComponent &) = delete;
    IComponent &operator=(const IComponent &) = delete;
    IComponent(IComponent &&) = delete;
    IComponent &operator=(IComponent &&) = delete;
    virtual ~IComponent() = default;
    struct OwnedBlock {
        std::string name;
        std::unique_ptr<IBlock> block;
    };
    [[nodiscard]] std::span<const OwnedBlock> blocks() const noexcept { return blocks_; }

protected:
    template <class B, class... Args>
        requires std::derived_from<B, IBlock> && std::constructible_from<B, Args...>
    B &makeBlock(std::string name, Args &&...args) {
        auto block = std::make_unique<B>(std::forward<Args>(args)...);
        auto &result = *block;
        blocks_.push_back({std::move(name), std::move(block)});
        return result;
    }

private:
    std::vector<OwnedBlock> blocks_;
};
} // namespace core
