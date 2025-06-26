import os
import random
import argparse
import numpy as np
import torch
import time
import math
from torch_geometric.data import Data
from torch_geometric.loader import DataLoader
from tqdm import tqdm
import typing as t

# --- Project-specific Imports ---
from core.map import Map
from core.node import GraphNode
from core.logger import logger
from algorithms.algorithms_implementations.prm import PRMAlgorithm
from algorithms.algorithms_implementations.learned_prm import GNNModel
from maps.maps_manager import MapsManager

# --- Constants ---
FEATURE_SIZE = 6
DEFAULT_MODEL_PATH = "gnn_model.pth"
DEFAULT_DATASET_PATH = "prm_dataset.pt"


def convert_prm_to_graph_data(prm: PRMAlgorithm) -> t.Optional[Data]:
    """Converts a solved PRM instance into a PyTorch Geometric Data object."""
    if not prm.is_complete() or not prm.shortest_path:
        return None

    all_nodes = prm.nodes
    node_to_idx = {node: i for i, node in enumerate(all_nodes)}

    map_diagonal = math.sqrt(prm.map.width ** 2 + prm.map.height ** 2)

    x_features, edge_list = [], []
    y_labels = torch.zeros(len(all_nodes), dtype=torch.float)
    path_nodes_set = set(prm.shortest_path)

    for node, idx in node_to_idx.items():
        norm_x, norm_y = node.x / prm.map.width, node.y / prm.map.height
        is_coll = 1.0 if prm.is_collision(node.x, node.y) else 0.0
        dist_s = prm.distance(node.get_position(), prm.start_node.get_position()) / map_diagonal
        dist_g = prm.distance(node.get_position(), prm.goal_node.get_position()) / map_diagonal
        degree = len(node.edges)
        x_features.append([norm_x, norm_y, is_coll, dist_s, dist_g, degree])

        for neighbor in node.edges:
            if neighbor in node_to_idx:
                edge_list.append([idx, node_to_idx[neighbor]])

        if node in path_nodes_set:
            y_labels[idx] = 1.0

    x = torch.tensor(x_features, dtype=torch.float)
    edge_index = torch.tensor(edge_list, dtype=torch.long).t().contiguous()

    if edge_index.shape[0] == 0 or y_labels.sum() == 0:
        return None

    return Data(x=x, edge_index=edge_index, y=y_labels.view(-1, 1))


def generate_dataset_from_maps(args):
    """Generates a dataset by running PRM on all available maps."""
    maps_manager = MapsManager()
    map_names = maps_manager.get_map_names()

    if not map_names:
        logger.error("MapsManager found no maps.")
        return

    logger.info(f"Found {len(map_names)} maps via MapsManager: {', '.join(map_names)}")
    data_list = []

    with tqdm(total=len(map_names) * args.runs_per_map, desc="Generating graph data") as pbar:
        for map_name in map_names:
            for _ in range(args.runs_per_map):
                try:
                    map_config = maps_manager.get_map(map_name)
                    if not map_config:
                        pbar.update(1)
                        continue

                    map_instance = Map(map_config.width, map_config.height, architecture="graph")
                    for obs in map_config.obstacles:
                        map_instance.add_obstacle(*obs)

                    prm = PRMAlgorithm(map=map_instance, num_samples_excluding_grid=args.prm_samples)

                    while True:
                        start_x = random.uniform(1, map_instance.width - 1)
                        start_y = random.uniform(1, map_instance.height - 1)
                        if not prm.is_collision(start_x, start_y):
                            map_instance.set_start(start_x, start_y)
                            prm.start_node = map_instance.start
                            break

                    while True:
                        goal_x = random.uniform(1, map_instance.width - 1)
                        goal_y = random.uniform(1, map_instance.height - 1)
                        if not prm.is_collision(goal_x, goal_y) and \
                                np.linalg.norm((start_x - goal_x, start_y - goal_y)) > map_instance.width / 4:
                            map_instance.set_goal(goal_x, goal_y)
                            prm.goal_node = map_instance.goal
                            break

                    start_time = time.time()
                    timeout = 10.0
                    max_steps = 10

                    while not prm.is_complete():
                        if time.time() - start_time > timeout: break
                        if prm.steps >= max_steps: break
                        prm.step()

                    graph_data = convert_prm_to_graph_data(prm)
                    if graph_data:
                        data_list.append(graph_data)

                except Exception as e:
                    logger.warning(f"Skipping run on map {map_name} due to an unexpected error: {e}")
                finally:
                    pbar.update(1)

    if not data_list:
        logger.error("Failed to generate any valid graph data. Consider increasing PRM samples or timeout.")
        return

    torch.save(data_list, args.dataset_path)
    logger.info(f"Dataset with {len(data_list)} graphs saved to {args.dataset_path}")


def train_model(args):
    """Main training loop for the GNN model."""
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    logger.info(f"Training on device: {device}")

    if not os.path.exists(args.dataset_path):
        logger.error(f"Dataset not found at '{args.dataset_path}'. Run in 'generate' mode first.")
        return

    data_list = torch.load(args.dataset_path, weights_only=False)

    random.shuffle(data_list)

    split_idx = int(len(data_list) * 0.8)
    train_loader = DataLoader(data_list[:split_idx], batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(data_list[split_idx:], batch_size=args.batch_size)

    model = GNNModel(in_channels=FEATURE_SIZE, hidden_channels=64, num_layers=3).to(device)

    # --- CHANGE: Load model weights if resuming training ---
    if args.resume:
        if os.path.exists(args.model_path):
            logger.info(f"Resuming training by loading model from '{args.model_path}'")
            model.load_state_dict(torch.load(args.model_path, map_location=device))
        else:
            logger.warning(f"Resume flag was set, but no model found at '{args.model_path}'. Starting from scratch.")

    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)

    num_pos = sum(d.y.sum() for d in data_list)
    num_total = sum(d.y.size(0) for d in data_list)
    pos_weight = (num_total - num_pos) / num_pos if num_pos > 0 else torch.tensor(1.0)
    loss_fn = torch.nn.BCEWithLogitsLoss(pos_weight=pos_weight.to(device))
    logger.info(f"Class Imbalance: Pos_weight = {pos_weight:.2f}")

    best_val_loss = float('inf')
    for epoch in range(args.epochs):
        model.train()
        total_loss = 0
        for batch in train_loader:
            batch.to(device)
            optimizer.zero_grad()
            out = model(batch.x, batch.edge_index)
            loss = loss_fn(out, batch.y)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        avg_train_loss = total_loss / len(train_loader)

        model.eval()
        total_val_loss = 0
        with torch.no_grad():
            for batch in val_loader:
                batch.to(device)
                out = model(batch.x, batch.edge_index)
                total_val_loss += loss_fn(out, batch.y).item()
        avg_val_loss = total_val_loss / len(val_loader)

        logger.info(f"Epoch {epoch + 1:02d}: Train Loss: {avg_train_loss:.4f}, Val Loss: {avg_val_loss:.4f}")

        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            torch.save(model.state_dict(), args.model_path)
            logger.info(f"  -> New best model saved to '{args.model_path}'")


def main():
    parser = argparse.ArgumentParser(description="GNN Training for PRM Sampler")
    parser.add_argument('mode', choices=['generate', 'train'], help="Mode: 'generate' dataset or 'train' model.")

    parser.add_argument('--dataset-path', type=str, default=DEFAULT_DATASET_PATH)
    parser.add_argument('--runs-per-map', type=int, default=10)
    parser.add_argument('--prm-samples', type=int, default=1000)

    parser.add_argument('--model-path', type=str, default=DEFAULT_MODEL_PATH)
    parser.add_argument('--epochs', type=int, default=75)
    parser.add_argument('--batch-size', type=int, default=32)
    parser.add_argument('--learning-rate', type=float, default=0.01)

    # --- CHANGE: Added the --resume flag ---
    parser.add_argument('--resume', action='store_true',
                        help="Resume training from the model specified by --model-path.")

    args = parser.parse_args()

    if args.mode == 'generate':
        generate_dataset_from_maps(args)
    elif args.mode == 'train':
        train_model(args)


if __name__ == "__main__":
    main()