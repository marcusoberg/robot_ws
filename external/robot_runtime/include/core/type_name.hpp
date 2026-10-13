#pragma once
#include <string_view>
#include <typeinfo>
namespace core {
// Human-readable graph metadata, without adding a demangling dependency to core.
template <class T> std::string_view typeName() noexcept {
#if defined(__clang__) || defined(__GNUC__)
    constexpr std::string_view signature = __PRETTY_FUNCTION__;
    constexpr auto begin = signature.find("T = ") + 4;
    return signature.substr(begin, signature.find(';', begin) != signature.npos ? signature.find(';', begin) - begin : signature.rfind(']') - begin);
#else
    return typeid(T).name();
#endif
}
} // namespace core
