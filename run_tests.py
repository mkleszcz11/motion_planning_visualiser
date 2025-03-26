from core.logger import logger

from test_runner.test_runner import TestRunner
from test_runner.test_analyse import TestAnalyser
from test_runner.combine_heatmaps import HeatmapCombiner

import os

if __name__ == "__main__":
    ### CONFIGURE TEST RUNNER ###
    test_runner = TestRunner(
        # algorithms=['RRT', 'Biased Random Walk'],
        # maps=['Simple Map', 'Simple Map V2'],
<<<<<<< Updated upstream
        # algorithms=['RRT','RRT, Biased','RRT*','RRT*, Biased'],
        algorithms=["RRT", "RRT*", "RRT*, V2"],
        maps=['Cluttered Map', 'Rooms'],
        runs_per_test = 10,
        step_size = 8.0,
=======
        algorithms=['RRT*'],
        maps=['Cluttered Map', 'Rooms'],
        runs_per_test = 10,
        step_size = 1.0,
>>>>>>> Stashed changes
        output_file="benchmark_results.csv"
    )

    ### RUN TESTS AND ANALYSIS ###
    logger.info("Running tests...")
    test_runner.run_tests()
    logger.info("Running test analysis...")
    
    analyser = TestAnalyser('benchmark_results.csv')
    analyser.generate_comparison_table()
    analyser.generate_heatmaps_v1()
    analyser.generate_heatmaps_v2()

    # TODO -> This is sketchy but I really have to go now. I'll fix it later.
    logger.info("Combining heatmaps...")
    this_dir = os.path.dirname(os.path.abspath(__file__))
    results_dir = f'{this_dir}/test_runner/results/'
    output_file = os.path.join(results_dir, 'combined_heatmaps.png')
    heatmap_combiner = HeatmapCombiner(results_dir=results_dir, output_file=output_file)
    heatmap_combiner.combine_heatmaps()
    # analyser.combine_heatmaps()
