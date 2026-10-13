#pragma once
#include <concepts>
#include <limits>
#include <stdexcept>

namespace robot::boundary {
struct Finite {
    static constexpr bool accepts(const double v) { return v >= -std::numeric_limits<double>::max() && v <= std::numeric_limits<double>::max(); }
};

struct Positive {
    static constexpr bool accepts(const double v) { return Finite::accepts(v) && v > 0; }
};

struct Fraction {
    static constexpr bool accepts(const double v) { return v > 0 && v <= 1; }
};

template <class T>
concept NumericDomain = requires(double v) {
    { T::accepts(v) } -> std::same_as<bool>;
};

// Validated at external input, or during translation for literal defaults. Once
// formed, a Number carries its domain without repeated checks by its consumers.
template <NumericDomain Domain> class Number {
public:
    consteval Number(double literal) : value_(literal) {
        if (!Domain::accepts(literal))
            throw "literal outside numeric domain";
    }

    [[nodiscard]] double value() const noexcept { return value_; }

    static Number read(double external) {
        if (!Domain::accepts(external))
            throw std::invalid_argument("number outside its required domain");
        return Number(external, Decoded{});
    }

private:
    struct Decoded {};

    Number(const double value, Decoded) : value_(value) {}

    double value_;
};

using FiniteNumber = Number<Finite>;
using PositiveNumber = Number<Positive>;
using UnitFraction = Number<Fraction>;
} // namespace robot::boundary
