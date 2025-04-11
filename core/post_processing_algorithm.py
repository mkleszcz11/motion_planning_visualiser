# core/post_processing_algorithm.py

from abc import ABC, abstractmethod
from typing import List, TYPE_CHECKING, Any

# Use forward reference for Algorithm to avoid circular import if needed
if TYPE_CHECKING:
    from core.algorithm import Algorithm
    from core.node import Node # Assuming path is list of Node-like objects

class PostProcessingAlgorithm(ABC):
    """Abstract base class for all path post-processing algorithms."""

    def __init__(self, **kwargs):
        """
        Initialize the algorithm. Can accept parameters via kwargs.
        Subclasses should call super().__init__(**kwargs) and handle
        their specific parameters.
        """
        pass # Base class might not need specific initialization

    @abstractmethod
    def process(self, original_path: List['Node'], algorithm_instance: 'Algorithm', **kwargs) -> List['Node']:
        """
        Processes the given path.

        Args:
            original_path: The list of Node objects representing the path found by the planning algorithm.
            algorithm_instance: The instance of the planning algorithm that generated the path,
                                used for accessing its collision checking methods.
            **kwargs: Additional parameters needed for processing (e.g., iterations, step_size).

        Returns:
            A list of Node objects representing the processed (e.g., smoothed, shortened) path.
            Returns the original path if no processing is done or possible.
        """
        pass

    def __str__(self):
        return self.__class__.__name__