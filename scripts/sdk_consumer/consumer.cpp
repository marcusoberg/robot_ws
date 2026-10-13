#include <robot_runtime/robot.hpp>
// Compile and link the public facade; this function is never executed.
void compileFacade(const robot_runtime::Configuration &configuration) {
    robot_runtime::Robot robot(configuration, "mujoco", robot_runtime::RunMode::Stepped);
    const auto source = robot.source("actor", "compile-consumer");
    auto acquisition = source->acquireControl();
    (void)acquisition.outcome();
    auto session = source->acquired(acquisition);
    (void)session.renew(robot.snapshot());
    (void)session.pause();
    robot_runtime::TimedJointTargets targets;
    auto submission = session.submit(targets, robot.snapshot());
    (void)submission.outcome();
    (void)robot.capture();
    (void)session.release();
}
int main() { return 0; }
