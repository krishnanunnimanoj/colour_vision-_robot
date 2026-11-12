#!/usr/bin/env python3

import argparse
import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory


COLORS = {
    "red": "1 0 0 1",
    "green": "0 1 0 1",
    "blue": "0 0 1 1",
    "yellow": "1 1 0 1",
}


def main():
    parser = argparse.ArgumentParser(
        description="Spawn a coloured sample object in Gazebo"
    )
    parser.add_argument(
        "--color",
        required=True,
        choices=COLORS.keys()
    )
    args = parser.parse_args()

    package_dir = get_package_share_directory(
        "color_pillar_description"
    )

    model_file = os.path.join(
        package_dir, "models", "colored_pillar", "model.sdf"
    )

    with open(model_file, "r", encoding="utf-8") as file:
        sdf = file.read()

    sdf = sdf.replace("1 0 0 1", COLORS[args.color])

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".sdf", delete=False
    ) as temp:
        temp.write(sdf)
        temp_path = temp.name

    try:
        command = [
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-entity", f"{args.color}_object",
            "-file", temp_path,
            "-x", "0.9",
            "-y", "0.0",
            "-z", "0.25"
        ]

        subprocess.run(command, check=True)
    finally:
        os.unlink(temp_path)


if __name__ == "__main__":
    main()
