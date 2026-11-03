"""Validate example model wiring without requiring a running simulator."""
import pathlib
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]


class ModelConfigurationTest(unittest.TestCase):
    def test_crab_completion_uses_physics_updates(self):
        source = (ROOT / "src/ackerman_drive_plugin.cpp").read_text()
        self.assertIn("event::Events::ConnectWorldUpdateBegin(", source)
        self.assertIn("std::bind(&AckermanDrivePlugin::OnUpdate, this)", source)
        update = source.split("void AckermanDrivePlugin::OnUpdate(){", 1)[1]
        update, odometry = update.split("void AckermanDrivePlugin::publishOdom(){", 1)
        self.assertIn("crabTransition_.Ready", update)
        self.assertIn("SimTime().Double()", update)
        self.assertIn("std::lock_guard<std::mutex>", update)
        self.assertNotIn("crabTransition_", odometry)

    def test_model_joint_references(self):
        for filename, six_wheel in [("sample_four_wheel.xacro", False),
                                    ("sample_six_wheel.xacro", True)]:
            with self.subTest(model=filename):
                root = ET.parse(ROOT / "urdf" / filename).getroot()
                joints = {joint.attrib["name"] for joint in root.findall("joint")}
                plugin = root.find("gazebo/plugin")
                self.assertIsNotNone(plugin)
                enabled = plugin.findtext("sixWheel", "false").lower()
                self.assertEqual(enabled in ("true", "1"), six_wheel)
                references = [item for item in plugin if item.tag.endswith("_joint")]
                self.assertEqual(len(references), 10 if six_wheel else 8)
                for item in references:
                    self.assertIn(item.text, joints)
                for dimension in ("wheelRadius", "wheelSeperation", "wheelBase"):
                    self.assertGreater(float(plugin.findtext(dimension)), 0)


if __name__ == "__main__":
    unittest.main()
