"""Check the motion node's balance switches after the machine YAML is loaded."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters, normalize_parameters


package = Path(sys.argv.pop(1))
spec = importlib.util.spec_from_file_location('bringup', package / 'launch/bringup.launch.py')
bringup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bringup)


class BringupBalanceTest(unittest.TestCase):
    def test_launch_switches_override_machine_config(self):
        static = 'wbc_static_com_enabled'
        dynamic = 'wbc_dynamic_zmp_enabled'
        contact = 'wbc_stationary_horizontal_contact_confirmed'
        machine_file = Path('/opt/onecube/motion_control/config/motion_control_config.yaml')
        for args, expected in [({}, (False, False)), ({static: 'true'}, (True, False)),
                               ({static: 'true', dynamic: 'true'}, (True, True))]:
            with self.subTest(args=args), patch.object(bringup, 'Node') as node, \
                    patch.object(bringup.FindPackageShare, 'find', return_value=str(package)):
                description = bringup.generate_launch_description()
                context = LaunchContext()
                context.launch_configurations.update(args)
                for entity in description.entities:
                    if isinstance(entity, DeclareLaunchArgument):
                        entity.execute(context)
                controller = next(call.kwargs for call in node.call_args_list
                                  if call.kwargs['executable'] == 'motion_control_node')
                effective = {}
                for layer in evaluate_parameters(context, normalize_parameters(controller['parameters'])):
                    if layer == machine_file:
                        effective.update({static: True, dynamic: True, contact: True})
                    elif isinstance(layer, dict):
                        effective.update(layer)
                self.assertIs(effective[static], expected[0])
                self.assertIs(effective[dynamic], expected[1])
                self.assertIs(effective[contact], True)


if __name__ == '__main__':
    unittest.main()
