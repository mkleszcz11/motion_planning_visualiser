# I allowed myself for a little magic in maps manager. I don't want to do
# it with algorithms as they might have a bit more complicated structure.
#
# Here I just want to keep it simple and scalable.
# For new algorithms, you just need to add a new file in algorithms directory,
# import it here and add it to the list of algorithms. Visualizer will handle the rest.

from algorithms.algorithms_implementations.random_walk import RandomWalkAlgorithm
from algorithms.algorithms_implementations.random_walk_biased import RandomWalkBiasedAlgorithm
from algorithms.algorithms_implementations.rrt import RRTAlgorithm
from algorithms.algorithms_implementations.rrt_biased import RRTBiasedAlgorithm
from algorithms.algorithms_implementations.rrt_connect import RRTConnectAlgorithm
from algorithms.algorithms_implementations.rrt_star import RRTStarAlgorithm
from algorithms.algorithms_implementations.rrt_star_biased import RRTStarBiasedAlgorithm
from algorithms.algorithms_implementations.prm_star_hybrid import HybridSamplingPRMStarAlgorithm

algorithms = [
    {
        "name": "Random Walk",
        "algorithm": RandomWalkAlgorithm
    },
    {
        "name": "Random Walk, Biased",
        "algorithm": RandomWalkBiasedAlgorithm
    },
    {
        "name": "RRT",
        "algorithm": RRTAlgorithm
    },
    {
        "name": "RRT-Connect",
        "algorithm": RRTConnectAlgorithm
    },
    {
        "name": "RRT*",
        "algorithm": RRTStarAlgorithm
    },
    {
        "name": "RRT, Biased",
        "algorithm": RRTBiasedAlgorithm
    },
    {
        "name": "RRT*, Biased",
        "algorithm": RRTStarBiasedAlgorithm
    },
    {
        "name": "PRM*, Hybrid",
        "algorithm": HybridSamplingPRMStarAlgorithm
    }
    
]

class AlgorithmManager:
    def __init__(self):
        self.algorithms = algorithms

    def get_algorithm_names(self):
        return [algorithm["name"] for algorithm in self.algorithms]

    def get_algorithm(self, name, map_instance, benchmark_manager):
        for algorithm in self.algorithms:
            if algorithm["name"] == name:
                return algorithm["algorithm"](map = map_instance,
                                              benchmark_manager = benchmark_manager)  # Instantiate directly
        return None
