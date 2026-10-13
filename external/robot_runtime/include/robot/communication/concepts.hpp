#pragma once
#include <concepts>
#include <utility>

#include "robot/backend/types.hpp"
#include "robot/communication/messages.hpp"
#include "robot/communication/types.hpp"
#include "robot/execution/controller_config.hpp"
#include "robot/execution/state.hpp"
#include "robot/resources/model.hpp"

namespace robot::communication {

// The complete compile-time contract used by communication adapters. Neither the
// concrete Execution nor direct-device control belongs to this interface.
template <class T>
concept Endpoint = requires(T &endpoint, communication::Command command, communication::OperatorRequest request) {
    { endpoint.description() } -> std::same_as<const communication::Description &>;
    { endpoint.model() } -> std::same_as<const robot::Model &>;
    { endpoint.controllerConfig() } -> std::same_as<const execution::ControllerConfig &>;
    { endpoint.capabilities() } -> std::same_as<backend::BackendCapabilities>;
    { endpoint.latest() } -> std::same_as<execution::Snapshot>;
    { endpoint.submit(std::move(command)) } -> std::same_as<bool>;
    { endpoint.renew(communication::AuthorityRenewal{}) } -> std::same_as<bool>;
    { endpoint.request(request) } -> std::same_as<bool>;
};

} // namespace robot::communication
