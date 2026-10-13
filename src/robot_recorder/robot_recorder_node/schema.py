"""ROS message-definition expansion for standard CDR views in canonical MCAP."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import re

_BUILTIN = {
    "bool",
    "byte",
    "char",
    "float32",
    "float64",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
    "string",
    "wstring",
}


def definition(type_name):
    seen = set()
    parts = []

    def visit(name, root=False):
        if name in seen:
            return
        seen.add(name)
        package, _, message = name.split("/")
        text = (
            Path(get_package_share_directory(package)) / "msg" / (message + ".msg")
        ).read_text()
        parts.append(
            text
            if root
            else "=" * 80 + "\nMSG: " + package + "/" + message + "\n" + text
        )
        for line in text.splitlines():
            line = line.split("#", 1)[0].strip()
            if not line or "=" in line:
                continue
            typename = re.split(r"[\[<]", line.split()[0])[0]
            if typename in _BUILTIN:
                continue
            dependency = (
                typename if "/" in typename else package + "/" + typename
            ).split("/")
            visit(dependency[0] + "/msg/" + dependency[-1])

    visit(type_name, True)
    return "\n".join(parts).encode()
