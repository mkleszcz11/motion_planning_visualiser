import unittest
from core.map import Map
from benchmarks.benchmark_manager import BenchmarkManager
from algorithms.algorithm_manager import AlgorithmManager
from maps.maps_manager import MapsManager
import time

class TestAlgorithmSolution(unittest.TestCase):
    """
    This validates that all algorithms can find a solution on all maps below 10 seconds
    (some can be excluded in code - like random walk).
    """
    def setUp(self):
        self.benchmark_manager = BenchmarkManager()
        self.algorithm_manager = AlgorithmManager()
        self.map_manager = MapsManager()

        # Define algorithms and maps to test
        self.algorithms = self.algorithm_manager.get_algorithm_names()
        self.maps = self.map_manager.get_map_names()

    def run_algorithm(self, algorithm_name, map_name, step_size=5, timeout=10):
        # Load map configuration
        map_config = self.map_manager.get_map(map_name)
        self.assertIsNotNone(map_config, f"Map '{map_name}' not found.")

        # Initialize map
        map_instance = Map(map_config.width, map_config.height)
        for obs in map_config.obstacles:
            map_instance.add_obstacle(*obs)

        # Set start and goal
        map_instance.set_start(*map_config.default_start)
        map_instance.set_goal(*map_config.default_goal)

        # Initialize algorithm
        algorithm = self.algorithm_manager.get_algorithm(
            algorithm_name,
            map_instance,
            self.benchmark_manager
        )
        algorithm.step_size = step_size

        self.assertIsNotNone(algorithm, f"Algorithm '{algorithm_name}' not found.")

        # Execute algorithm until completion or timeout
        start_time = time.time()
        while not algorithm.is_complete():
            algorithm.step()
            if time.time() - start_time > timeout:
                self.fail(f"{algorithm_name} failed to find a solution on '{map_name}' within {timeout} seconds.")

        # Validate results
        result = self.benchmark_manager.get_last_result()
        self.assertIsNotNone(result, f"No benchmark result for {algorithm_name} on '{map_name}'")
        self.assertGreater(len(result.path), 0, f"{algorithm_name} produced an empty path on '{map_name}'")
        self.assertLessEqual(result.execution_time, timeout, f"{algorithm_name} exceeded timeout on '{map_name}'")
        self.assertGreater(result.path_length, 0, f"{algorithm_name} produced a path with zero length on '{map_name}'")

        # Last node should be step_size close to goal
        distance_to_goal = algorithm.distance(result.path[-1].get_position(), map_instance.goal.get_position())
        self.assertLessEqual(distance_to_goal, step_size,
                             f"Last node of {algorithm_name} path is not close to goal on '{map_name}'")

    def test_all_algorithms(self):
        """Test all algorithms on all maps. Define some custom rules for specific cases."""
        for algorithm in self.algorithms:
            for map_name in self.maps:
                with self.subTest(algorithm=algorithm, map_name=map_name):
                    print(f"Testing {algorithm} on {map_name}")
                    if algorithm == "Random Walk":
                        print("\t- Skipping Random Walk as it is very slow.")
                        continue
                    elif map_name == "Narrow Passage":
                        print("\t- Skipping Narrow Passage due to very long execution time.")
                        continue
                    else:
                        self.run_algorithm(algorithm, map_name)

if __name__ == "__main__":
    unittest.main()
