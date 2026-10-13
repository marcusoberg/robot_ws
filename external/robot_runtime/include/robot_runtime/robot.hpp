#pragma once
#include "application/runtime.hpp"
#include "application/operations.hpp"

namespace robot_runtime {
using Configuration=application::DeploymentSpecification;
using Observation=robot::execution::Snapshot;
using Capture=application::Capture;
using ImageSample=application::ImageSample;
using Source=application::Source;
using TimedJointTargets=robot::communication::TimedJointTargetsRequest;
using RunMode=application::RunMode;
using Robot=application::Runtime;
using Submission=application::Submission;
using ControlSession=application::ControlSession;
using Operation=application::Operation;
using MotionOperation=application::MotionOperation;
using OperationResult=application::OperationResult;
using Outcome=application::Outcome;
// C++ users load specifications before construction; Python Configuration maps
// the same types. Execution and numerical Controller remain private to Robot.
inline Configuration configuration(const std::filesystem::path &path) { return application::loadSpecification(path); }
} // namespace robot_runtime
