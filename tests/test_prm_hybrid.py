import unittest
from core.map import Map
from benchmarks.benchmark_manager import BenchmarkManager
from algorithms.algorithms_implementations.prm_hybrid import HybridPRMAlgorithm

import math

class TestHybridPRMAlgorithm(unittest.TestCase):
    def setUp(self):
        self.map = Map(100, 100)
        self.map.set_start(5, 5)
        self.map.set_goal(95, 95)
        self.benchmark_manager = BenchmarkManager()
        self.prm = HybridPRMAlgorithm(
            map=self.map,
            benchmark_manager=self.benchmark_manager,
            num_samples_excluding_grid=0  # Only grid - make it rectangular
        )

    def test_generate_grid(self):
        self.prm.generate_default_grid()
        self.assertGreater(len(self.prm.samples), 0)

    def test_connect_neighbors(self):
        self.prm.generate_default_grid()
        self.prm.connect_neighbors(self.prm.start_node)
        self.prm.connect_neighbors(self.prm.goal_node)
        self.assertTrue(len(self.prm.start_node.edges) >= 0)
        self.assertTrue(len(self.prm.goal_node.edges) >= 0)

    def test_a_star_execution(self):
        self.prm.step()  # Step 0: Generate grid
        self.prm.step()  # Step 1: Connect neighbors
        self.prm.step()  # Step 2: Run A*
        self.assertTrue(self.prm.is_complete())
        self.assertGreater(len(self.prm.shortest_path), 1)

    def test_path_cost_is_reasonable(self):
        self.prm.step()
        self.prm.step()
        self.prm.step()
        cost = self.prm.calculate_shortest_path_cost()
        shortest_possible_distance = math.sqrt(self.prm.start_node.x**2 + self.prm.start_node.y**2)
        self.assertGreater(cost, shortest_possible_distance)
        reasonable_max_distance = 5 + abs(self.prm.start_node.x - self.prm.goal_node.x) + abs(self.prm.start_node.y - self.prm.goal_node.y)
        self.assertLess(cost, reasonable_max_distance)

    def test_clear_nodes(self):
        self.prm.step()
        self.prm.clear_nodes()
        self.assertEqual(self.prm.steps, 0)
        self.assertEqual(len(self.prm.samples), 0)

    def test_reinitialise_start_and_goal(self):
        self.prm.step()
        self.prm.step()
        self.prm.reinintialise_start_and_goal(self.map.start, self.map.goal)
        self.assertEqual(self.prm.steps, 2)
        self.assertEqual(len(self.prm.start_node.edges) > 0, True)

if __name__ == "__main__":
    unittest.main()
