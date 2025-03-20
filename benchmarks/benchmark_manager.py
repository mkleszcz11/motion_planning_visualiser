from core.logger import logger

class BenchmarkManager:
    def __init__(self):
        self.results = []

    def add_result(self, result):
        self.results.append(result)

    def print_results(self):
        for result in self.results:
            logger.info(result)

    def get_last_result(self):
        if not self.results:
            return None
        return self.results[-1]

    def clear_results(self):
        self.results = []