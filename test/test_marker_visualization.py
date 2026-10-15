"""Latest-plan replacement and complete RViz snapshots, without robot commands."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import MagicMock

from visualization_msgs.msg import Marker, MarkerArray


path = Path(__file__).resolve().parents[1] / "scripts/manipulation_marker_visualization.py"
spec = importlib.util.spec_from_file_location("manipulation_marker_visualization", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def plan(source, roles):
    markers = []
    for role, count in roles:
        for index in range(count):
            marker = Marker()
            marker.ns = source + "/" + role
            marker.id = index
            marker.header.frame_id = "robot/base_link"
            marker.frame_locked = True
            marker.type = Marker.LINE_STRIP if "path" in role else Marker.ARROW
            marker.pose.orientation.w = 1.0
            markers.append(marker)
    return MarkerArray(markers=markers)


def apply_snapshot(display, snapshot):
    for marker in snapshot.markers:
        if marker.action == Marker.DELETEALL:
            # Match RViz MarkerCommon: a nonempty namespace scopes DELETEALL.
            if marker.ns:
                for key in list(display):
                    if key[0] == marker.ns:
                        del display[key]
            else:
                display.clear()
        elif marker.action == Marker.ADD:
            display[(marker.ns, marker.id)] = marker


class MarkerVisualizationTest(unittest.TestCase):
    def setUp(self):
        self.cartesian = plan("cartesian", [("planned_path", 1), ("desired_path", 1),
                                            ("target", 3), ("final", 3)])
        self.bimanual = plan("bimanual_cartesian", [("arm_a_path", 1), ("arm_b_path", 1),
            ("planned_path", 1), ("desired_path", 1),
            ("target", 3), ("arm_a_final", 3), ("arm_b_final", 3)])

    def test_clear_and_complete_plan_share_one_message_with_unique_keys(self):
        for source in (self.cartesian, self.bimanual):
            snapshot = module.replacement_snapshot(source)
            self.assertEqual(snapshot.markers[0].action, Marker.DELETEALL)
            self.assertEqual(snapshot.markers[0].ns, "")
            keys = [(marker.ns, marker.id) for marker in snapshot.markers]
            self.assertEqual(len(keys), len(set(keys)))
            self.assertEqual(snapshot.markers[1:], source.markers)
            self.assertTrue(all(marker.frame_locked for marker in snapshot.markers[1:]))
            self.assertTrue(all(marker.action == Marker.ADD for marker in source.markers))

    def test_cartesian_then_bimanual_then_cartesian_replaces_all_previous_layers(self):
        display = {}
        for source in (self.cartesian, self.bimanual, self.cartesian):
            apply_snapshot(display, module.replacement_snapshot(source))
            self.assertEqual(set(display), {(marker.ns, marker.id) for marker in source.markers})

    def test_pose_snapshot_replaces_joint_paths_on_the_common_output(self):
        node = MagicMock()
        display = {}
        pose = plan("cartesian", [("desired_path", 1), ("target", 3)])
        for source in (self.cartesian, pose, self.bimanual, pose):
            module.ManipulationMarkerVisualization._on_plan(node, source)
            snapshot = node._publisher.publish.call_args.args[0]
            apply_snapshot(display, snapshot)
            self.assertEqual(set(display), {(marker.ns, marker.id) for marker in source.markers})
        self.assertEqual(node._publisher.publish.call_count, 4)
        self.assertEqual(len(display), 4)
        node.get_logger.return_value.warning.assert_not_called()

    def test_last_message_alone_reconstructs_complete_display(self):
        node = MagicMock()
        module.ManipulationMarkerVisualization._on_plan(node, self.cartesian)
        module.ManipulationMarkerVisualization._on_plan(node, self.bimanual)
        self.assertEqual(node._publisher.publish.call_count, 2)
        last_message = node._publisher.publish.call_args.args[0]
        late_display = {}
        apply_snapshot(late_display, last_message)
        self.assertEqual(len(late_display), 13)
        self.assertTrue(all(key[0].startswith("bimanual_cartesian/") for key in late_display))

    def test_duplicate_input_is_rejected_without_overwriting_valid_plan(self):
        node = MagicMock()
        module.ManipulationMarkerVisualization._on_plan(node, self.cartesian)
        invalid = MarkerArray(markers=[self.bimanual.markers[0], self.bimanual.markers[0]])
        module.ManipulationMarkerVisualization._on_plan(node, invalid)
        self.assertEqual(node._publisher.publish.call_count, 1)
        node.get_logger.return_value.warning.assert_called_once()

    def test_reserved_key_and_input_delete_are_rejected(self):
        marker = Marker()
        marker.ns = module.CLEAR_NAMESPACE
        marker.id = module.CLEAR_ID
        with self.assertRaises(ValueError):
            module.replacement_snapshot(MarkerArray(markers=[marker]))
        marker.ns = "cartesian/target"
        marker.action = Marker.DELETE
        with self.assertRaises(ValueError):
            module.replacement_snapshot(MarkerArray(markers=[marker]))

    def test_namespaced_deleteall_does_not_clear_other_namespaces(self):
        display = {(marker.ns, marker.id): marker for marker in self.cartesian.markers}
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.ns = "_sura_visualization_clear"
        apply_snapshot(display, MarkerArray(markers=[clear]))
        self.assertEqual(len(display), 8)
        clear.ns = "cartesian/target"
        apply_snapshot(display, MarkerArray(markers=[clear]))
        self.assertEqual(len(display), 5)
        self.assertTrue(all(ns != "cartesian/target" for ns, unused in display))

    def test_empty_namespace_add_with_other_id_does_not_conflict_with_clear(self):
        marker = Marker()
        marker.id = 0
        snapshot = module.replacement_snapshot(MarkerArray(markers=[marker]))
        keys = [(item.ns, item.id) for item in snapshot.markers]
        self.assertEqual(len(keys), len(set(keys)))
        self.assertEqual(snapshot.markers[0].ns, "")
        self.assertNotEqual(snapshot.markers[0].id, marker.id)

    def test_empty_plan_explicitly_clears_display(self):
        display = {}
        apply_snapshot(display, module.replacement_snapshot(self.cartesian))
        snapshot = module.replacement_snapshot(MarkerArray())
        self.assertEqual(len(snapshot.markers), 1)
        apply_snapshot(display, snapshot)
        self.assertEqual(len(display), 0)


if __name__ == "__main__":
    unittest.main()
