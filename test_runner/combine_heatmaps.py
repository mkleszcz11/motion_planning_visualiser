import os
from PIL import Image, ImageDraw, ImageFont

class HeatmapCombiner:
    def __init__(self, results_dir, output_file):
        self.results_dir = results_dir
        self.output_file = output_file
        self.heatmaps = self.load_heatmaps()

    def load_heatmaps(self):
        files = os.listdir(self.results_dir)
        heatmaps = {}

        for file in files:
            if file.startswith('heatmap_') and file.endswith('.png'):
                if file.startswith('heatmap_v2_'):
                    key = file.replace('heatmap_v2_', '').replace('.png', '') + "_v2"
                else:
                    key = file.replace('heatmap_', '').replace('.png', '')
                
                heatmaps[key] = os.path.join(self.results_dir, file)

        return heatmaps

    def combine_heatmaps(self):
        if not self.heatmaps:
            print("No heatmaps found.")
            return

        # Get list of maps and algorithms
        keys = list(self.heatmaps.keys())
        map_names = sorted(set([key.split('_')[1] for key in keys]))
        algorithms = sorted(set([key.split('_')[0] for key in keys]))

        # Load sample image to determine size
        sample_image = Image.open(list(self.heatmaps.values())[0])
        img_width, img_height = sample_image.size

        padding = 20
        label_height = 20

        for map_name in map_names:
            combined_width = 2 * img_width + 3 * padding
            combined_height = len(algorithms) * (img_height + label_height + padding) + padding

            # Create a blank canvas for each map
            canvas = Image.new("RGB", (combined_width, combined_height), "white")
            draw = ImageDraw.Draw(canvas)

            # Load font (if available)
            try:
                font = ImageFont.truetype("arial.ttf", 14)
            except IOError:
                font = ImageFont.load_default()

            for row, algorithm in enumerate(algorithms):
                # ✅ Fix key handling for consistency
                normal_key = f"{algorithm}_{map_name}"
                v2_key = f"{algorithm}_{map_name}_v2"

                y_offset = row * (img_height + label_height + padding) + padding

                # Add label above each row
                draw.text((padding, y_offset), f"{algorithm} - Normal", fill="black", font=font)
                draw.text((img_width + 2 * padding, y_offset), f"{algorithm} - V2", fill="black", font=font)

                # Paste Normal image
                if normal_key in self.heatmaps:
                    normal_image = Image.open(self.heatmaps[normal_key])
                    canvas.paste(normal_image, (padding, y_offset + label_height))

                # Paste V2 image
                if v2_key in self.heatmaps:
                    v2_image = Image.open(self.heatmaps[v2_key])
                    canvas.paste(v2_image, (img_width + 2 * padding, y_offset + label_height))

            # Save combined image for this map
            output_file = os.path.join(self.results_dir, f"combined_heatmap_{map_name}.png")
            canvas.save(output_file)
            print(f"Combined heatmap saved to {output_file}")

# if __name__ == "__main__":
#     combiner = HeatmapCombiner()
#     combiner.combine_heatmaps()
