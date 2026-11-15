import unittest

from analyze_base_arm_dynamics import analyze


class AnalyzeBaseArmDynamicsTest(unittest.TestCase):
    def test_reports_pose_displacement_and_keeps_missing_data_explicit(self):
        rows = [
            {"source": "dynamic_pose", "sim_time": {"seconds": 1.0},
             "payload": {"watched_poses": [
                 {"name": "robot", "position": {"x": 0, "y": 0, "z": 0},
                  "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}
             ]}},
            {"source": "dynamic_pose", "sim_time": {"seconds": 2.0},
             "payload": {"watched_poses": [
                 {"name": "robot", "position": {"x": 0.03, "y": 0.04, "z": 0},
                  "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}
             ]}},
            {"source": "joint_state", "sim_time": {"seconds": 2.0},
             "payload": {"watched_joints": [
                 {"name": "left_wheel_joint", "axis1": {"velocity": 0.5}},
                 {"name": "L_elbow_joint", "axis1": {"velocity": -1.2}}
             ]}},
        ]
        result = analyze(rows)
        self.assertAlmostEqual(
            result["pose_displacement"]["robot"]["max_xy_displacement_from_first_m"],
            0.05,
        )
        self.assertEqual(result["source_coverage"]["joint_state"]["records"], 1)
        self.assertNotIn("contact_state", result["source_coverage"])
        self.assertAlmostEqual(
            result["joint_velocity_abs_rad_s"]["wheel"]["max_abs"], 0.5
        )
        self.assertAlmostEqual(
            result["joint_velocity_abs_rad_s"]["arm"]["max_abs"], 1.2
        )

    def test_empty_capture_is_explicit_not_a_success(self):
        result = analyze([])
        self.assertEqual(result["record_count"], 0)
        self.assertEqual(result["pose_displacement"], {})
        self.assertIsNone(result["base_command_abs_components"]["max_abs"])


if __name__ == "__main__":
    unittest.main()
