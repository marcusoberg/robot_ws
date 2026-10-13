#pragma once
#include <array>
#include <ranges>
#include <stdexcept>

namespace robot::boundary {
// Sequential decoding establishes arity at the boundary without exposing a
// resizeable container to consumers. Shared by deployment and transport codecs.
template <std::ranges::input_range Range> class Sequence {
public:
    explicit Sequence(Range &range) : cursor_(std::ranges::begin(range)), end_(std::ranges::end(range)) {}

    decltype(auto) next() {
        if (cursor_ == end_)
            throw std::invalid_argument("missing input coordinate");
        return *cursor_++;
    }

    void finish() const {
        if (cursor_ != end_)
            throw std::invalid_argument("unexpected input coordinate");
    }

private:
    std::ranges::iterator_t<Range> cursor_;
    std::ranges::sentinel_t<Range> end_;
};

template <class T, std::size_t N, std::ranges::input_range Range> std::array<T, N> readArray(Range &&range) {
    Sequence input{range};
    std::array<T, N> result;
    for (auto &value : result)
        value = input.next();
    input.finish();
    return result;
}
} // namespace robot::boundary
