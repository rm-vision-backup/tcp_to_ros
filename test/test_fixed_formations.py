#!/usr/bin/env python3
import os
import importlib.util
import sys
import unittest

SCRIPT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts", "gcs_a_backend_node.py"))
SPEC = importlib.util.spec_from_file_location("gcs_a_backend_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
GcsABackendNode = MODULE.GcsABackendNode


class Item:
    def __init__(self, uav_id): self.uav_id = uav_id


class Goal:
    command = "FOLLOW_ROUTE"
    def __init__(self, ids): self.assignments = [Item(value) for value in ids]


class FixedFormationTest(unittest.TestCase):
    def setUp(self):
        path = os.path.join(os.path.dirname(__file__), "..", "config", "formation_offsets.yaml")
        (self.offsets, self.participants, self.leader,
         self.anchor) = GcsABackendNode._load_formation_offsets(os.path.abspath(path))
        self.node = object.__new__(GcsABackendNode)
        self.node.formation_participants = self.participants

    def test_exact_sets_select_unique_slots(self):
        ids1 = ["A%02d" % i for i in range(1, 13)]
        ids2 = ["A%02d" % i for i in range(1, 7)] + ["A13", "A14", "A15"]
        self.assertEqual(self.node._formation_for_goal(Goal(ids1)), "formation_1")
        self.assertEqual(self.node._formation_for_goal(Goal(ids2)), "formation_2")
        self.assertEqual(self.leader, "A01")
        self.assertEqual(self.anchor, (10.0, 5.0, 0.0))

    def test_missing_member_is_rejected(self):
        self.assertIsNone(self.node._formation_for_goal(Goal(["A%02d" % i for i in range(1, 12)])))


if __name__ == "__main__":
    unittest.main()
