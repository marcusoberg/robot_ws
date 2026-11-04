"""Stop the local robot server without looking up its process ID."""

import sys

from robot_server_node.ownership import stop_existing_server


def main():
    try:
        stopped = stop_existing_server()
        print("Robot server stopped" if stopped else "Robot server is already stopped")
        return 0
    except (OSError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
