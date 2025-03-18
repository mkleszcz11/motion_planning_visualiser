# I allowed myself for a little magic in maps manager. I don't want to do
# it with algorithms as they might have a bit more complicated structure.
#
# Here I just want to keep it simple and scalable.
# For new algorithms, you just need to add a new file in algorithms directory,
# import it here and add it to the list of algorithms. Visualizer will handle the rest.

from algorithms.algorithms_implementations.random_walk import RandomWalkAlgorithm
from algorithms.algorithms_implementations.random_walk_biased import RandomWalkBiasedAlgorithm

algorithms = [
    {
        "name": "Random Walk",
        "algorithm": RandomWalkAlgorithm
    },
    {
        "name": "Biased Random Walk",
        "algorithm": RandomWalkBiasedAlgorithm
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
                return algorithm["algorithm"](map_instance, benchmark_manager)  # Instantiate directly
        return None