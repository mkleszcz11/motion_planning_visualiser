from core.algorithm import Algorithm

from algorithms.algorithms_implementations.random_walk import RandomWalkAlgorithm
from algorithms.algorithms_implementations.random_walk_biased import RandomWalkBiasedAlgorithm
from algorithms.algorithms_implementations.rrt import RRTAlgorithm
from algorithms.algorithms_implementations.rrt_biased import RRTBiasedAlgorithm
from algorithms.algorithms_implementations.rrt_connect import RRTConnectAlgorithm
from algorithms.algorithms_implementations.rrt_star import RRTStarAlgorithm
from algorithms.algorithms_implementations.rrt_star_biased import RRTStarBiasedAlgorithm

algorithms = [
    {
        "name": "RRT-Connect",
        "algorithm": RRTConnectAlgorithm
    },
    {
        "name": "RRT",
        "algorithm": RRTAlgorithm
    },
    {
        "name": "RRT - Biased",
        "algorithm": RRTBiasedAlgorithm
    },
    {
        "name": "RRT*",
        "algorithm": RRTStarAlgorithm
    },
    {
        "name": "RRT* - Biased",
        "algorithm": RRTStarBiasedAlgorithm
    },
    {
        "name": "Random Walk",
        "algorithm": RandomWalkAlgorithm
    },
    {
        "name": "Random Walk - Biased",
        "algorithm": RandomWalkBiasedAlgorithm
    },
]

class AlgorithmManager:
    def __init__(self):
        self.algorithms = algorithms

    def get_algorithm_names(self):
        return [algorithm["name"] for algorithm in self.algorithms]

    def get_algorithm(self, name, map_instance, benchmark_manager) -> Algorithm:
        for algorithm in self.algorithms:
            if algorithm["name"] == name:
                return algorithm["algorithm"](map = map_instance,
                                              benchmark_manager = benchmark_manager)  # Instantiate directly
        return None
