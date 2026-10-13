#pragma once
// Public architecture: resources describe the robot; Communication exchanges
// messages; Execution owns authority/planning/control; Backend owns the device.
#include "robot/backend/concepts.hpp"
#include "robot/communication/concepts.hpp"
#include "robot/execution/state.hpp"
#include "robot/resources/model.hpp"

namespace robot {
using State = execution::Snapshot;
template <class T>
concept BackendAdapter = backend::Device<T>;
template <class T>
concept CommunicationAdapter = communication::Endpoint<T>;
class Communication;
class Execution;
class Backend;
} // namespace robot
