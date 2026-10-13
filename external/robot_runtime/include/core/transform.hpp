#pragma once
#include "core/Component.hpp"
#include <functional>
namespace core {
// A named single-operation component. The callable and registry are resolved at construction.
template <SignalData Out, SignalData... Ins> class Transform : public Component<Inputs<Ins...>, Outputs<Out>> {
    using Function = std::function<Out(const Context &, const Ins &...)>;
    class Operation final : public Block {
    public:
        std::tuple<std::unique_ptr<InPort<Ins>>...> in;
        OutPort<Out> out;
        Operation(Out initial, Function fn) : in(std::make_unique<InPort<Ins>>(*this)...), out(*this, std::move(initial)), fn_(std::move(fn)) {}
        void evaluate(const Context &ctx) override {
            out.set(std::apply([&](auto &...port) { return fn_(ctx, port->get()...); }, in));
        }

    private:
        Function fn_;
    };

public:
    Transform(Out initial, std::tuple<Ins...> inputs, Function fn) : Component<Inputs<Ins...>, Outputs<Out>>(std::move(inputs), std::tuple{initial}) {
        auto &operation = this->template makeBlock<Operation>("evaluate", std::move(initial), std::move(fn));
        connect(operation, std::index_sequence_for<Ins...>{});
        operation.out >> this->template output<Out>();
    }

private:
    template <std::size_t... I> void connect(Operation &op, std::index_sequence<I...>) {
        ((this->template input<std::tuple_element_t<I, std::tuple<Ins...>>>() >> *std::get<I>(op.in)), ...);
    }
};
template <SignalData T> class Delay final : public Component<Inputs<>, Outputs<T>> {
    class Operation final : public Block {
    public:
        FeedbackPort<T> in;
        OutPort<T> out;
        explicit Operation(T initial) : in(*this, initial), out(*this, std::move(initial)) {}
        void evaluate(const Context &) override { out.set(in.get()); }
    };

public:
    explicit Delay(T initial)
        : Component<Inputs<>, Outputs<T>>(std::tuple{}, std::tuple{initial}), operation_(this->template makeBlock<Operation>("delay", std::move(initial))) {
        operation_.out >> this->template output<T>();
    }
    FeedbackPort<T> &input() { return operation_.in; }

private:
    Operation &operation_;
};
} // namespace core
