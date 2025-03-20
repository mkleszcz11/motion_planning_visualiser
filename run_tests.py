from test_runner.test_runner import TestRunner
import os

if __name__ == "__main__":
    test_runner = TestRunner(
        # algorithms=['RRT', 'Biased Random Walk'],
        # maps=['Simple Map', 'Simple Map V2'],
        algorithms=['RRT','RRT, Biased','RRT*','RRT*, Biased'],
        maps=['Cluttered Map', 'Rooms'],
        runs_per_test = 10,
        step_size = 5.0,
        output_file="benchmark_results.csv"
    )

    RESULTS_DIR = 'test_runner/results/'
    # Purge results directory
    for file in os.listdir(RESULTS_DIR):
        os.remove(os.path.join(RESULTS_DIR, file))

    test_runner.run_tests()
