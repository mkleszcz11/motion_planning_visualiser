# post_processing/post_processing_manager.py

from typing import List, Type, Optional

# Import the base class
from core.post_processing_algorithm import PostProcessingAlgorithm
from post_processing.algorithms.critical_point_bspline import CriticalPointBSpline
from post_processing.algorithms.hybrid_bspline import HybridBSplineSmoothing

# Import specific algorithm implementations
from post_processing.algorithms.path_shortening import PathShortening
# --- IMPORT THE NEW ALGORITHM ---
from post_processing.algorithms.bspline_smoothing import BSplineSmoothing
# ---------------------------------

# Define the list of available algorithms
available_algorithms = [
    {
        "name": "Path Shortening (Random)",
        "algorithm": PathShortening
    },
    # --- ADD THE NEW ALGORITHM ---
    {
        "name": "B-Spline Smoothing (Quintic)", # Name for the dropdown
        "algorithm": BSplineSmoothing
    },
    {
        "name": "Hybrid Quintic B-Spline Smoothing",  # Name for the dropdown
        "algorithm": HybridBSplineSmoothing  # Use the renamed class
    },
    {
        "name": "Critical Point B-Spline",  # Name for the dropdown
        "algorithm": CriticalPointBSpline  # Use the new class
    },
    # ----------------------------
]

class PostProcessingManager:
    """Manages the registration and retrieval of post-processing algorithms."""

    def __init__(self):
        self._algorithms = available_algorithms
        # Optional: Validate that all registered items are subclasses of PostProcessingAlgorithm
        for algo_info in self._algorithms:
            if not issubclass(algo_info["algorithm"], PostProcessingAlgorithm):
                raise TypeError(f"Algorithm {algo_info['name']} ({algo_info['algorithm']}) is not a subclass of PostProcessingAlgorithm.")

    def get_algorithm_names(self) -> List[str]:
        """Returns a list of display names for the registered algorithms."""
        return [algo["name"] for algo in self._algorithms]

    def get_algorithm_class(self, name: str) -> Optional[Type[PostProcessingAlgorithm]]:
        """
        Retrieves the class for the post-processing algorithm with the given name.

        Args:
            name: The display name of the algorithm.

        Returns:
            The algorithm class if found, otherwise None.
        """
        for algo_info in self._algorithms:
            if algo_info["name"] == name:
                return algo_info["algorithm"]
        return None