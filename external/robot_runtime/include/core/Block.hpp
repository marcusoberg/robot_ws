#pragma once
#include "core/interfaces.hpp"
namespace core {
class Block : public IBlock {
public:
    Block() = default;
    // Connected ports borrow member storage; blocks must retain their addresses.
    Block(const Block &) = delete;
    Block &operator=(const Block &) = delete;
    Block(Block &&) = delete;
    Block &operator=(Block &&) = delete;
    void registerInput(IInPort &port) { inputs_.push_back(&port); }
    [[nodiscard]] std::span<IInPort *const> inputs() const noexcept final { return inputs_; }
    void reset() override {
        for (auto *port : inputs_)
            port->reset();
    }

private:
    std::vector<IInPort *> inputs_;
};
} // namespace core
