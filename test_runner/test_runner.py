import csv
import os
import time
from core.logger import logger
from core.map import Map
from core.node import Node
import json
from algorithms.algorithm_manager import AlgorithmManager
from benchmarks.benchmark_manager import BenchmarkManager


class TestRunner:
    def __init__(self, algorithms, maps, runs_per_test, output_file, step_size=5):
        self.algorithms = algorithms
        self.maps = maps
        self.runs_per_test = runs_per_test
        self.output_file = os.path.join('test_runner/results/', output_file)
        self.step_size = step_size
        self.algorithm_manager = AlgorithmManager()
        self.benchmark_manager = BenchmarkManager()

        # Ensure results directory exists
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

    def run_tests(self):
        logger.info(f"Starting tests: {self.runs_per_test} runs per algorithm-map pair")
        with open(self.output_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Algorithm", "Map", "Execution Time", "Path Length", "Steps", "Path"])

            for map_name in self.maps:
                for algorithm_name in self.algorithms:
                    for run in range(self.runs_per_test):
                        logger.info(f"Running {algorithm_name} on {map_name} (Run {run + 1}/{self.runs_per_test})")
                        result = self.run_single_test(algorithm_name, map_name)
                        if result:
                            writer.writerow([
                                result.algorithm_name,
                                map_name,
                                f"{result.execution_time:.4f}",
                                f"{result.path_length:.2f}",
                                result.steps,
                                self.serialize_path(result.path)
                            ])

                        self.reset_environment()

    def run_single_test(self, algorithm_name, map_name):
        map_config = self.get_map(map_name)
        if not map_config:
            logger.warning(f"Map '{map_name}' not found.")
            return None
        
        map_instance = Map(map_config.width, map_config.height)
        for obs in map_config.obstacles:
            map_instance.add_obstacle(*obs)

        map_instance.set_start(*map_config.default_start)
        map_instance.set_goal(*map_config.default_goal)

        algorithm = self.algorithm_manager.get_algorithm(
            algorithm_name,
            map_instance,
            self.benchmark_manager
        )
        algorithm.step_size = self.step_size
        if algorithm is None:
            logger.error(f"Algorithm '{algorithm_name}' not found.")
            return None

        start_time = time.time()
        timeout = 10  # 10 seconds timeout

        while not algorithm.is_complete():
            algorithm.step()
            if time.time() - start_time > timeout:
                logger.warning(f"Timeout reached for {algorithm_name} on {map_name}")
                return None
        
        result = self.benchmark_manager.get_last_result()
        if result:
            logger.info(f"{algorithm_name} completed on {map_name} in {result.execution_time:.4f}s")
        return result

    def serialize_path(self, path):
        if not path:
            return ""
        return json.dumps([(node.x, node.y) for node in path])

    def get_map(self, map_name):
        from maps.maps_manager import MapsManager
        maps_manager = MapsManager()
        return maps_manager.get_map(map_name)

    def reset_environment(self):
        logger.info("Resetting environment...")
        self.benchmark_manager.clear_results()
