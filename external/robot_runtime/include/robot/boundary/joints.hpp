#pragma once
#include "robot/boundary/numbers.hpp"
#include "robot/boundary/sequence.hpp"
#include "robot/resources/model.hpp"

namespace robot::boundary {
// Consume an external sequence into exactly the coordinates of one joint space.
// There is no arbitrary-vector constructor on a domain joint value. Cardinality
// is established here at decoding, never checked again in execution.
template <resources::JointQuantity T, std::ranges::input_range Range> T readJoints(const Model &model, Range &&external) {
    Sequence source{external};
    auto result = model.joints().generate<T>([&](Eigen::Index index) {
        const auto value = FiniteNumber::read(source.next()).value();
        if constexpr (std::same_as<T, resources::Positions>) {
            if (value < model.description().lower[index] || value > model.description().upper[index])
                throw std::invalid_argument("joint target outside configured limits");
        }
        return value;
    });
    source.finish();
    return result;
}
} // namespace robot::boundary
