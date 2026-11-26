import importlib.util
import math
import os
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml


ROS2 = Path(__file__).resolve().parents[1]
DESCRIPTION = ROS2 / "src/astro_robot_description"
NAVIGATION = ROS2 / "src/astro_navigation"
DESCRIPTION_LAUNCHES = (
    ROS2 / "src/robot_bringup/launch/robot_base.launch.py",
    DESCRIPTION / "launch/astro_robot_description.launch.py",
)


class Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class LaunchDescription:
    def __init__(self, actions=None):
        self.actions = actions or []

    def add_action(self, action):
        self.actions.append(action)


def module(name, **attributes):
    result = types.ModuleType(name)
    result.__dict__.update(attributes)
    return result


class RobotModelTest(unittest.TestCase):
    def setUp(self):
        modules = {
            "launch": module("launch", LaunchDescription=LaunchDescription),
            "launch.actions": module(
                "launch.actions",
                DeclareLaunchArgument=Action,
                IncludeLaunchDescription=Action,
            ),
            "launch_ros": module("launch_ros"),
            "launch_ros.actions": module("launch_ros.actions", Node=Action),
            "launch.substitutions": module(
                "launch.substitutions",
                LaunchConfiguration=Action,
                PythonExpression=Action,
            ),
            "launch.launch_description_sources": module(
                "launch.launch_description_sources", PythonLaunchDescriptionSource=Action
            ),
            "launch.conditions": module("launch.conditions", IfCondition=Action),
            "ament_index_python": module("ament_index_python"),
            "ament_index_python.packages": module(
                "ament_index_python.packages",
                get_package_share_directory=lambda package: str(
                    NAVIGATION if package == "astro_navigation2" else DESCRIPTION
                ),
            ),
        }
        self.modules = patch.dict(sys.modules, modules)
        self.modules.start()
        self.addCleanup(self.modules.stop)
    def load_launch(self, path):
        spec = importlib.util.spec_from_file_location("robot_launch", path)
        launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launch)
        return launch

    def test_selected_model_is_published(self):
        for path in DESCRIPTION_LAUNCHES:
            for value, expected in ((None, "waffle"), ("waffle", "waffle"), ("burger", "burger")):
                with self.subTest(path=path, value=value), patch.dict(os.environ, {}, clear=True):
                    if value is not None:
                        os.environ["ASTRO_ROBOT_MODEL"] = value
                    with patch("os.path.exists", return_value=True):
                        description = self.load_launch(path).generate_launch_description()
                    publisher = next(
                        action
                        for action in description.actions
                        if action.kwargs.get("name") == "robot_state_publisher"
                    )
                    urdf = DESCRIPTION / "urdf" / f"{expected}.urdf"
                    self.assertEqual(publisher.kwargs["arguments"], [str(urdf)])
                    self.assertEqual(
                        publisher.kwargs["parameters"][0]["robot_description"],
                        urdf.read_text(),
                    )

    def test_navigation_selects_model_and_preserves_override(self):
        path = NAVIGATION / "launch/navigation2.launch.py"
        for value, expected in ((None, "waffle"), ("waffle", "waffle"), ("burger", "burger")):
            with self.subTest(value=value), patch.dict(os.environ, {}, clear=True):
                if value is not None:
                    os.environ["ASTRO_ROBOT_MODEL"] = value
                description = self.load_launch(path).generate_launch_description()
                argument = next(
                    action
                    for action in description.actions
                    if action.args and action.args[0] == "params_file"
                )
                config = argument.kwargs["default_value"]
                self.assertEqual(config.args[0], "params_file")
                self.assertEqual(config.kwargs["default"], str(NAVIGATION / "param" / f"{expected}.yaml"))
                include = next(
                    action
                    for action in description.actions
                    if "launch_arguments" in action.kwargs
                )
                self.assertIs(dict(include.kwargs["launch_arguments"])["params_file"], config)

    def test_invalid_model_is_rejected(self):
        paths = (*DESCRIPTION_LAUNCHES, NAVIGATION / "launch/navigation2.launch.py")
        for path in paths:
            with self.subTest(path=path), patch.dict(os.environ, {"ASTRO_ROBOT_MODEL": "../burger"}):
                with self.assertRaisesRegex(ValueError, "Unsupported robot model"):
                    self.load_launch(path).generate_launch_description()

    def test_burger_meshes_resolve_in_description_package(self):
        root = ET.parse(DESCRIPTION / "urdf/burger.urdf").getroot()
        meshes = root.findall(".//mesh")
        self.assertTrue(meshes)
        for mesh in meshes:
            uri = mesh.attrib["filename"]
            prefix = "package://astro_robot_description/"
            self.assertTrue(uri.startswith(prefix), uri)
            self.assertTrue((DESCRIPTION / uri[len(prefix):]).is_file(), uri)

    def test_burger_costmaps_cover_documented_dimensions(self):
        params = yaml.safe_load((NAVIGATION / "param/burger.yaml").read_text())
        local = params["local_costmap"]["local_costmap"]["ros__parameters"]["robot_radius"]
        global_radius = params["global_costmap"]["global_costmap"]["ros__parameters"]["robot_radius"]
        self.assertEqual(local, global_radius)
        self.assertGreaterEqual(local, math.hypot(0.138 / 2 + 0.032, 0.178 / 2))


if __name__ == "__main__":
    unittest.main()
