import os
import random
import torch
import torch.nn as nn
import torch.nn.functional as F
import typing as t
import math

from core.map import Map
from core.node import GraphNode
from core.logger import logger
from benchmarks.benchmark_manager import BenchmarkManager

from algorithms.algorithms_implementations.prm import PRMAlgorithm

try:
    from torch_geometric.nn import GCNConv
except ImportError:
    logger.critical("PyTorch Geometric not found. The LearnedPRMAlgorithm will not be available.")
    logger.critical("Please install it: pip install torch_geometric")
    GCNConv = object


class GNNModel(torch.nn.Module):
    def __init__(self, in_channels: int, hidden_channels: int, num_layers: int = 3):
        super().__init__()
        self.convs = torch.nn.ModuleList()
        self.convs.append(GCNConv(in_channels, hidden_channels))
        for _ in range(num_layers - 1):
            self.convs.append(GCNConv(hidden_channels, hidden_channels))
        self.classifier = nn.Linear(hidden_channels, 1)

    def forward(self, x: torch.Tensor, edge_index: torch.Tensor) -> torch.Tensor:
        for conv in self.convs:
            x = conv(x, edge_index)
            x = F.leaky_relu(x)
        return self.classifier(x)


class LearnedPRMAlgorithm(PRMAlgorithm):
    # --- CHANGE: Made __init__ more robust to handle different ways it might be called ---
    def __init__(self, *args, **kwargs):
        # Pass all arguments to the parent PRMAlgorithm constructor
        super().__init__(*args, **kwargs)

        # Attributes specific to this learned version
        self.model_path = "gnn_model.pth"
        self.num_candidates = 2000  # Number of random points to evaluate in each step
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.feature_size = 6

        if not os.path.exists(self.model_path):
            logger.error(f"FATAL: GNN model not found at '{self.model_path}'.")
            raise FileNotFoundError(f"Model file not found: {self.model_path}")

        self.model = GNNModel(in_channels=self.feature_size, hidden_channels=64).to(self.device)
        self.model.load_state_dict(torch.load(self.model_path, map_location=self.device))
        self.model.eval()
        # This log message is now correctly placed after the super().__init__ call
        # logger.info(f"Loaded GNN model from '{self.model_path}' onto {self.device}")

    def _create_features_for_inference(self, temp_nodes: t.List[GraphNode],
                                       temp_edges: t.Dict[int, t.List[int]]) -> torch.Tensor:
        features = []
        map_diagonal = math.sqrt(self.map.width ** 2 + self.map.height ** 2)
        for i, node in enumerate(temp_nodes):
            norm_x = node.x / self.map.width
            norm_y = node.y / self.map.height
            is_collision = 1.0 if self.is_collision(node.x, node.y) else 0.0
            dist_start = self.distance(node.get_position(), self.start_node.get_position()) / map_diagonal
            dist_goal = self.distance(node.get_position(), self.goal_node.get_position()) / map_diagonal
            degree = len(temp_edges.get(i, []))
            features.append([norm_x, norm_y, is_collision, dist_start, dist_goal, degree])

        return torch.tensor(features, dtype=torch.float32, device=self.device)

    @torch.no_grad()
    def generate_points_on_the_map(self):
        logger.info("--- Starting GNN-guided sampling ---")

        logger.info("[DEBUG] Step 1/8: Generating candidate nodes.")
        # --- BUG FIX: Manually generate random points instead of calling a non-existent method ---
        candidate_nodes = []
        for _ in range(self.num_candidates):
            rand_x = random.uniform(0, self.map.width)
            rand_y = random.uniform(0, self.map.height)
            candidate_nodes.append(GraphNode(rand_x, rand_y))
        logger.info(f"[DEBUG] Generated {len(candidate_nodes)} candidate nodes.")

        existing_nodes = self.samples[:]
        temp_nodes = existing_nodes + candidate_nodes
        logger.info(
            f"[DEBUG] Step 2/8: Created temporary graph with {len(temp_nodes)} nodes ({len(existing_nodes)} existing + {len(candidate_nodes)} candidates).")

        logger.info("[DEBUG] Step 3/8: Building temporary edge list...")
        edge_list = []
        temp_edges_for_degree: t.Dict[int, list] = {i: [] for i in range(len(temp_nodes))}
        # --- PERFORMANCE FIX: Avoid O(n^2) loop to prevent stack overflow crash ---
        for i, node1 in enumerate(temp_nodes):
            # Check against a random subset of other nodes
            check_indices = random.sample(range(len(temp_nodes)), k=min(150, len(temp_nodes)))
            for j in check_indices:
                if i == j: continue
                node2 = temp_nodes[j]

                if self.distance(node1.get_position(), node2.get_position()) <= self.neighbour_radius:
                    if not self.is_edge_collision(node1.x, node1.y, node2.x, node2.y):
                        edge_list.extend([[i, j], [j, i]])
                        temp_edges_for_degree[i].append(j)
                        temp_edges_for_degree[j].append(i)
        logger.info(f"[DEBUG] Built edge list with {len(edge_list)} directed edges.")

        logger.info("[DEBUG] Step 4/8: Creating feature tensor for PyTorch.")
        x = self._create_features_for_inference(temp_nodes, temp_edges_for_degree)
        edge_index = torch.tensor(edge_list, dtype=torch.long, device=self.device).t().contiguous()
        logger.info(f"[DEBUG] Created feature tensor of shape {x.shape} and edge_index of shape {edge_index.shape}.")

        logger.info("[DEBUG] Step 5/8: Running GNN model inference...")
        scores = self.model(x, edge_index)
        logger.info("[DEBUG] GNN inference complete.")

        logger.info("[DEBUG] Step 6/8: Processing and sorting results.")
        candidate_scores = scores[len(existing_nodes):].squeeze()

        valid_candidates = []
        for i, cand_node in enumerate(candidate_nodes):
            if x[len(existing_nodes) + i, 2].item() == 0.0:
                valid_candidates.append((cand_node, candidate_scores[i].item()))

        valid_candidates.sort(key=lambda item: item[1], reverse=True)
        logger.info(f"[DEBUG] Found {len(valid_candidates)} valid (collision-free) candidates.")

        logger.info("[DEBUG] Step 7/8: Adding best candidates to the official roadmap.")
        num_to_add = min(len(valid_candidates), self.num_samples)
        self.samples.extend([node for node, score in valid_candidates[:num_to_add]])
        logger.info(f"[DEBUG] Added {num_to_add} new samples to the map.")

        logger.info("[DEBUG] Step 8/8: GNN-guided sampling finished successfully.")
        logger.info("--- ---")