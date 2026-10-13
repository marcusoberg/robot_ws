#pragma once
#include "core/channels.hpp"
#include "core/type_name.hpp"
#include <tuple>
#include <typeindex>
#include <unordered_map>
namespace core {
template <class InputSchema, class OutputSchema> class Component;
template <class... Ins, class... Outs> class Component<Inputs<Ins...>, Outputs<Outs...>> : public IComponent {
    struct Holder {
        virtual ~Holder() = default;
    };
    template <class Channel> struct TypedHolder : Holder {
        explicit TypedHolder(Channel channel) : value(std::move(channel)) {}
        Channel value;
    };

protected:
    Component(std::tuple<Ins...> inputs, std::tuple<Outs...> outputs) {
        std::apply([this](auto &&...value) { (add<InputChannel>(inputs_, std::move(value)), ...); }, std::move(inputs));
        std::apply([this](auto &&...value) { (add<OutputChannel>(outputs_, std::move(value)), ...); }, std::move(outputs));
    }

public:
    template <class T>
        requires((std::same_as<T, Ins>) || ...)
    InputChannel<T> &input() {
        return lookup<InputChannel<T>>(inputs_, typeid(T));
    }
    template <class T>
        requires((std::same_as<T, Outs>) || ...)
    OutputChannel<T> &output() {
        return lookup<OutputChannel<T>>(outputs_, typeid(T));
    }

private:
    using Registry = std::unordered_map<std::type_index, std::unique_ptr<Holder>>;
    template <template <class> class Channel, class T> void add(Registry &registry, T initial) {
        constexpr auto direction = std::same_as<Channel<T>, InputChannel<T>> ? "in: " : "out: ";
        auto &block = this->template makeBlock<ChannelBlock<T>>(std::string(direction) + std::string(typeName<T>()), std::move(initial));
        registry.emplace(typeid(T), std::make_unique<TypedHolder<Channel<T>>>(Channel<T>{block}));
    }
    template <class Channel> static Channel &lookup(Registry &registry, std::type_index type) {
        return static_cast<TypedHolder<Channel> &>(*registry.at(type)).value;
    }
    Registry inputs_, outputs_;
};
} // namespace core
