#pragma once
#include <boost/sml.hpp>
#include <sstream>

namespace robot::execution::automaton {
struct Enable {};
struct Disable {};
struct Stop {};
struct Fault {};
struct Run {};
struct Complete {};
struct Resume {};
struct Enabled {
    auto operator()() const {
        using namespace boost::sml;
        return make_transition_table(*"Holding"_s + event<Run> = "Running"_s, "Running"_s + event<Run> = "Running"_s,
                                     "Running"_s + event<Complete> = "Holding"_s);
    }
};
struct Machine {
    auto operator()() const {
        using namespace boost::sml;
        return make_transition_table(
            *"Disabled"_s + event<Enable> = "Stopping"_s, "Faulted"_s + event<Enable> = "Stopping"_s, "Stopping"_s + event<Complete> = state<Enabled>,
            "Stopping"_s + event<Resume> = state<Enabled>, state<Enabled> + event<Stop> = "Stopping"_s, "Stopping"_s + event<Stop> = "Stopping"_s,
            "Disabled"_s + event<Stop> = "Disabled"_s, "Faulted"_s + event<Stop> = "Faulted"_s, state<Enabled> + event<Disable> = "Disabled"_s,
            "Stopping"_s + event<Disable> = "Disabled"_s, "Faulted"_s + event<Disable> = "Disabled"_s, "Disabled"_s + event<Disable> = "Disabled"_s,
            state<Enabled> + event<Fault> = "Faulted"_s, "Stopping"_s + event<Fault> = "Faulted"_s, "Disabled"_s + event<Fault> = "Faulted"_s,
            "Faulted"_s + event<Fault> = "Faulted"_s);
    }
};
// Export the actual transition types, including the enabled submachine.
template <class T> std::string name() {
    std::string value = boost::sml::aux::string<T>{}.c_str();
    const auto scope = value.rfind("::");
    if (scope != std::string::npos)
        value.erase(0, scope + 2);
    if (const auto end = value.find('>'); end != std::string::npos)
        value.erase(end);
    return value;
}
template <class... Transitions> void edges(boost::sml::aux::type_list<Transitions...>, std::ostream &out) {
    const auto edge = [&]<class T>() {
        if constexpr (T::initial)
            out << "[*] --> " << name<typename T::src_state>() << '\n';
        out << name<typename T::src_state>() << " --> " << name<typename T::dst_state>() << " : " << name<typename T::event>() << '\n';
    };
    (edge.template operator()<Transitions>(), ...);
}
inline std::string diagram() {
    std::ostringstream out;
    out << "@startuml\n";
    edges(typename boost::sml::sm<Machine>::transitions{}, out);
    out << "state Enabled {\n";
    edges(typename boost::sml::sm<Enabled>::transitions{}, out);
    out << "}\n@enduml\n";
    return out.str();
}
} // namespace robot::execution::automaton
