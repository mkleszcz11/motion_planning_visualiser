from test_runner.test_runner import TestRunner

if __name__ == "__main__":
    test_runner = TestRunner(
        # algorithms=['RRT', 'Biased Random Walk'],
        # maps=['Simple Map', 'Simple Map V2'],
        algorithms=['RRT', 'Biased Random Walk'],
        maps=['Cluttered Map', 'Rooms Map'],
        runs_per_test=100,
        step_size=5,
        output_file="benchmark_results.csv"
    )

    test_runner.run_tests()
