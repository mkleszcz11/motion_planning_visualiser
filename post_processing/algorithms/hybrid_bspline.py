# post_processing/algorithms/hybrid_bspline.py

import numpy as np
import math
from typing import List, TYPE_CHECKING

try:
    import scipy.interpolate as si
except ImportError:
    si = None

from core.post_processing_algorithm import PostProcessingAlgorithm
from core.logger import logger
from core.node import Node

if TYPE_CHECKING:
    from core.algorithm import Algorithm

# --- RENAMED CLASS ---
class HybridBSplineSmoothing(PostProcessingAlgorithm):
    """
    Post-processing algorithm that smooths a path using B-splines.
    Attempts to create a hybrid path using spline segments where they are
    collision-free, reverting to original path nodes where spline segments collide.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if si is None:
             logger.error("HybridBSplineSmoothing requires SciPy. Please install it (`pip install scipy`).")

    def _map_spline_index_to_original(self, spline_index: int, num_spline_points: int, num_original_points: int) -> int:
        """Approximates the index of the original path node closest to a given spline point's parameter."""
        if num_original_points <= 1: return 0
        original_index = (spline_index / max(1, num_spline_points - 1)) * (num_original_points - 1)
        return min(int(round(original_index)), num_original_points - 1) # Round and clamp

    def process(self, original_path: List[Node], algorithm_instance: 'Algorithm', **kwargs) -> List[Node]:
        """
        Applies B-spline smoothing and creates a hybrid collision-checked path.

        Args:
            original_path: The input path (list of Node objects).
            algorithm_instance: The planning algorithm instance for collision checks.
            **kwargs: Should contain 'num_samples' (int) for the number of points on the output spline
                      and optionally 'degree' (int, default 5).

        Returns:
            A hybrid path containing collision-free spline segments and original nodes,
            or the original path if smoothing fails.
        """
        # (Check for si, path length, algorithm_instance as before)
        if si is None: logger.error("Cannot smooth, SciPy not installed."); return original_path
        if not original_path or len(original_path) < 2: logger.debug("Path too short."); return original_path
        if not algorithm_instance: logger.error("Algorithm instance required."); return original_path

        num_samples = kwargs.get('num_samples')
        degree = kwargs.get('degree', 5) # Default Quintic

        # (Parameter checks as before)
        if num_samples is None: logger.error("'num_samples' parameter required."); return original_path
        if num_samples < 2: logger.warning("'num_samples' < 2, using 2."); num_samples = 2


        logger.info(f"Running Hybrid BSplineSmoothing (deg={degree}, {num_samples} samples)...")

        # --- Generate Full B-Spline Points ---
        # (Spline generation logic remains exactly the same as previous version)
        full_spline_nodes = []
        num_original = len(original_path)
        try:
            control_vertices = np.array([[n.x, n.y] for n in original_path])
            count = control_vertices.shape[0]
            safe_degree = np.clip(degree, 1, max(1, count - 1))
            if safe_degree != degree: logger.warning(f"Degree clipped: {degree} -> {safe_degree} (CPs={count}).")
            if count <= safe_degree: logger.warning(f"Not enough CPs ({count}) for degree {safe_degree}. Returning original."); return original_path
            denominator = count - safe_degree
            if denominator <= 0: logger.error(f"Invalid state: count={count}, degree={safe_degree}"); return original_path
            kv = np.array([0]*safe_degree + list(range(count-safe_degree+1)) + [count-safe_degree]*safe_degree, dtype='float')
            kv /= denominator
            u = np.linspace(0.0, 1.0, num_samples)
            tck = (kv, control_vertices.T, safe_degree)
            smoothed_points_np = np.array(si.splev(u, tck)).T
            for point in smoothed_points_np: full_spline_nodes.append(Node(point[0], point[1]))
        except Exception as e: logger.error(f"Error during B-spline calculation: {e}", exc_info=True); return original_path
        if not full_spline_nodes: logger.error("B-spline calculation resulted in empty path."); return original_path

        logger.info(f"Generated {len(full_spline_nodes)} spline points. Reconstructing hybrid path...")

        # --- Reconstruct Hybrid Path ---
        # (Hybrid path reconstruction logic remains exactly the same as previous version)
        hybrid_path = []
        if not full_spline_nodes: return original_path
        hybrid_path.append(full_spline_nodes[0])
        last_added_node_type = 'spline'
        collision_segments_count = 0
        num_spline_segments = len(full_spline_nodes) - 1

        for j in range(num_spline_segments):
            spline_node_start = full_spline_nodes[j]; spline_node_end = full_spline_nodes[j+1]
            has_collision = False
            try: has_collision = algorithm_instance.is_edge_collision(spline_node_start.x, spline_node_start.y, spline_node_end.x, spline_node_end.y)
            except AttributeError: logger.error(f"Collision checker missing."); return original_path
            except Exception as e: logger.error(f"Collision check error: {e}"); return original_path

            if not has_collision: # Spline segment is clear
                if not hybrid_path or (hybrid_path[-1].x != spline_node_end.x or hybrid_path[-1].y != spline_node_end.y):
                     hybrid_path.append(spline_node_end); last_added_node_type = 'spline'
            else: # Spline segment collides
                collision_segments_count += 1; logger.debug(f"Spline segment {j} collides. Reverting.")
                original_node_index = self._map_spline_index_to_original(j + 1, len(full_spline_nodes), num_original)
                original_node_to_add = original_path[original_node_index]
                if not hybrid_path or (hybrid_path[-1].x != original_node_to_add.x or hybrid_path[-1].y != original_node_to_add.y):
                    last_valid_node = hybrid_path[-1]; bridge_collision = False
                    try: bridge_collision = algorithm_instance.is_edge_collision(last_valid_node.x, last_valid_node.y, original_node_to_add.x, original_node_to_add.y)
                    except Exception as e: logger.error(f"Bridge check error: {e}"); return original_path
                    if not bridge_collision: hybrid_path.append(original_node_to_add); last_added_node_type = 'original'
                    else: logger.error(f"Bridge collision to original node {original_node_index}. Reverting."); return original_path

        # (Final Goal Check logic remains exactly the same as previous version)
        original_goal_node = original_path[-1]
        if hybrid_path and (hybrid_path[-1].x != original_goal_node.x or hybrid_path[-1].y != original_goal_node.y):
             last_hybrid_node = hybrid_path[-1]; final_bridge_collision = False
             try: final_bridge_collision = algorithm_instance.is_edge_collision(last_hybrid_node.x, last_hybrid_node.y, original_goal_node.x, original_goal_node.y)
             except Exception as e: logger.error(f"Final bridge check error: {e}"); return original_path
             if not final_bridge_collision: logger.debug("Appending original goal node."); hybrid_path.append(original_goal_node)
             else: logger.error("Final bridge collision. Reverting."); return original_path

        if collision_segments_count > 0: logger.warning(f"Hybrid BSpline: {collision_segments_count}/{num_spline_segments} spline segments collided.")
        else: logger.info("Hybrid BSpline: All spline segments were collision-free.")

        # (Duplicate Removal logic remains exactly the same as previous version)
        final_path = []
        if hybrid_path:
            final_path.append(hybrid_path[0])
            for i in range(1, len(hybrid_path)):
                if hybrid_path[i].x != hybrid_path[i-1].x or hybrid_path[i].y != hybrid_path[i-1].y: final_path.append(hybrid_path[i])

        logger.info(f"Hybrid BSplineSmoothing finished. Final path has {len(final_path)} points.")
        return final_path