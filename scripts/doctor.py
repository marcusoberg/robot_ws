"""Static installation check: no robot, recorder or model processes are started."""

from pathlib import Path
import importlib
import json
import sys
from ament_index_python.packages import get_package_share_directory
from rosidl_generator_py import import_type_support

ROOT = Path(__file__).resolve().parents[1]


def main():
    errors = []
    modules = {}
    packages = {}
    for name in (
        "robot_runtime",
        "robot_runtime_native",
        "robot_runtime_native._native",
        "robot_inference",
        "robot_inference.codec",
        "robot_server_node",
        "robot_examples",
        "robot_teleop",
        "model_client",
        "robot_server_node.node",
        "robot_server_client",
        "robot_server_client.producer",
        "robot_examples.operator",
        "robot_examples.send_joint",
        "robot_examples.send_timed_targets",
        "robot_examples.boundary_probe",
        "robot_teleop.keyboard",
        "model_client.client",
    ):
        try:
            module = importlib.import_module(name)
            path = Path(module.__file__).resolve()
            modules[name] = str(path)
            if not path.is_relative_to(ROOT) or any(
                p in path.parts
                for p in (
                    "archive",
                    "parol6_demo",
                    "robot-experiments",
                    "policy_runner",
                    "robot-data",
                    "robot-ros",
                )
            ):
                errors.append(f"{name} resolves outside the fresh layout: {path}")
        except Exception as error:
            errors.append(f"{name}: {error}")
    for name in (
        "robot_server",
        "robot_examples",
        "robot_teleop",
        "model_client",
        "robot_bringup",
    ):
        try:
            path = Path(get_package_share_directory(name)).resolve()
            packages[name] = str(path)
            if not path.is_relative_to(ROOT / "install/ros"):
                errors.append(
                    f"{name} package resources resolve outside fresh ROS installation: {path}"
                )
        except Exception as error:
            errors.append(f"{name}: {error}")
    for name in ("robot_server",):
        try:
            import_type_support(name)
        except Exception as error:
            errors.append(f"{name} typesupport: {error}")
    try:
        from robot_server_node.sdk_compatibility import validate_sdk_usage

        errors.extend(validate_sdk_usage())
    except Exception as error:
        errors.append(f"SDK facade compatibility: {error}")
    report = {
        "python": sys.executable,
        "modules": modules,
        "ros_packages": packages,
        "errors": errors,
        "operational_smoke_run": False,
    }
    print(json.dumps(report, indent=2))
    return bool(errors)


if __name__ == "__main__":
    raise SystemExit(main())
