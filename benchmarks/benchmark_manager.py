class BenchmarkManager:
    def __init__(self):
        self.results = []

    def add_result(self, result):
        self.results.append(result)

    def print_results(self):
        for result in self.results:
            print(result)

    def export_to_csv(self, filename="benchmark_results.csv"):
        import csv
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Algorithm", "Path Length", "Steps", "Execution Time (s)"])
            
            for result in self.results:
                if result.algorithm_name is None or result.path_length is None or result.steps is None or result.execution_time is None:
                    print("ERR - Result is missing data!")
                else:
                    writer.writerow([result.algorithm_name, result.path_length, result.steps, result.execution_time])

