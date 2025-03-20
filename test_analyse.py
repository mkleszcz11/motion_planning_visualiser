import os
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from maps.maps_manager import MapsManager
from matplotlib.patches import Rectangle
import numpy as np

RESULTS_DIR = 'test_runner/results/'

class TestAnalyser:
    def __init__(self, filename):
        self.filepath = os.path.join(RESULTS_DIR, filename)
        self.data = pd.read_csv(self.filepath)
        self.maps_manager = MapsManager()

    def generate_comparison_table(self):
        summary = self.data.groupby(['Algorithm', 'Map']).agg(
            mean_time=('Execution Time', 'mean'),
            var_time=('Execution Time', 'var'),
            mean_length=('Path Length', 'mean'),
            var_length=('Path Length', 'var'),
            mean_steps=('Steps', 'mean'),
            var_steps=('Steps', 'var')
        ).reset_index()

        print("\n=== Comparison Table ===")
        print(summary)
        output_file = os.path.join(RESULTS_DIR, 'comparison_table.csv')
        summary.to_csv(output_file, index=False)
        print(f"Comparison table saved to {output_file}")

    def generate_heatmaps_v1(self):
        for algorithm in self.data['Algorithm'].unique():
            for map_name in self.data['Map'].unique():
                subset = self.data[(self.data['Algorithm'] == algorithm) & (self.data['Map'] == map_name)]

                plt.figure(figsize=(8, 8))
                all_x, all_y = [], []
                
                # ✅ Load map and obstacles
                map_config = self.maps_manager.get_map(map_name)
                if map_config:
                    self.draw_obstacles(map_config)

                for path in subset['Path']:
                    points = eval(path)
                    xs, ys = zip(*points)
                    all_x.extend(xs)
                    all_y.extend(ys)

                # Create density-based heatmap
                sns.kdeplot(x=all_x, y=all_y, cmap='Blues', fill=True, alpha=0.7)
                plt.title(f"{algorithm} - {map_name}")
                plt.xlim(0, map_config.width)
                plt.ylim(0, map_config.height)

                output_path = os.path.join(RESULTS_DIR, f"heatmap_{algorithm}_{map_name}.png")
                plt.savefig(output_path)
                plt.close()

                print(f"Heatmap saved to {output_path}")
                
    def generate_heatmaps_v2(self):
        for algorithm in self.data['Algorithm'].unique():
            for map_name in self.data['Map'].unique():
                subset = self.data[(self.data['Algorithm'] == algorithm) & (self.data['Map'] == map_name)]

                map_config = self.maps_manager.get_map(map_name)
                if not map_config or len(subset) == 0:
                    continue

                grid_size = 100
                grid = np.zeros((grid_size, grid_size))

                plt.figure(figsize=(8, 8))

                # ✅ Step 1: Build density grid
                for path in subset['Path']:
                    points = eval(path)
                    for x, y in points:
                        x_idx = int(x)
                        y_idx = int(y)
                        if 0 <= x_idx < grid_size and 0 <= y_idx < grid_size:
                            grid[y_idx][x_idx] += 1

                # ✅ Step 2: Normalize density grid
                if grid.max() > 0:
                    grid = grid / grid.max()

                # ✅ Step 3: Plot density-based heatmap using imshow()
                plt.imshow(
                    grid,
                    cmap="Blues",
                    origin="upper",
                    extent=[0, map_config.width, map_config.height, 0],
                    alpha=0.5
                )

                # ✅ Step 4: Overlay individual paths using plt.plot()
                alpha = np.clip(1 / len(subset), 0.1, 0.6)  # Ensure alpha range is valid

                for path in subset['Path']:
                    points = eval(path)
                    xs, ys = zip(*points)
                    plt.plot(xs, ys, color='blue', alpha=alpha, linewidth=1.2)

                # ✅ Step 5: Draw obstacles after rendering paths and heatmap
                self.draw_obstacles(map_config)

                # ✅ Step 6: Finalize heatmap
                plt.title(f"{algorithm} - {map_name}")
                plt.xlim(0, map_config.width)
                plt.ylim(0, map_config.height)

                output_path = os.path.join(RESULTS_DIR, f"heatmap_v2_{algorithm}_{map_name}.png")
                plt.savefig(output_path)
                plt.close()

                print(f"Heatmap v2 saved to {output_path}")

    def draw_obstacles(self, map_config):
        ax = plt.gca()

        for ox, oy, w, h in map_config.obstacles:
            rect = Rectangle(
                (ox, oy), w, h,
                linewidth=1,
                edgecolor='black',
                facecolor='gray',
                alpha=0.6
            )
            ax.add_patch(rect)

if __name__ == "__main__":
    analyser = TestAnalyser('benchmark_results.csv')
    analyser.generate_comparison_table()
    analyser.generate_heatmaps_v1()
    analyser.generate_heatmaps_v2()
