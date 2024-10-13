import time
from functools import wraps

def timing_decorator(on_start, on_end):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            self = args[0]  # Extract `self` from the arguments of the method

            # Call the start function with `self`
            if callable(on_start):
                on_start(self)

            start_time = time.time()
            result = func(*args, **kwargs)
            end_time = time.time()
            execution_time = end_time - start_time

            # Call the end function with `self` and the execution time
            if callable(on_end):
                on_end(self, execution_time)

            return result
        return wrapper
    return decorator


# def start_message():
#     print("Function is starting...")

# def end_message(execution_time):
#     print(f"Function finished. Execution time: {execution_time:.4f} seconds")

@timing_decorator(
    lambda: print("Function is starting..."),
    lambda ex_time: print(f"Function finished. Execution time: {ex_time:.4f} seconds"))
def my_function(a, b):
    print(f"Function arguments: {a}, {b}")
    time.sleep(2)  # Simulate a time-consuming task
    print("Doing some work...")

# my_function(1, 2)


# Example class
class MyClass:
    def __init__(self, logger):
        self.logger = logger

    @timing_decorator(
        lambda self: self.logger.info('Loading config...'),
        lambda self, ex_time: self.logger.info(f'Loaded config in {ex_time:.4f} seconds.')
    )
    def my_function(self):
        time.sleep(2)  # Simulate a time-consuming task
        print("Config loaded.")

# Example logger with a simple print function
class Logger:
    @staticmethod
    def info(message):
        print(message)

# Usage
logger = Logger()
obj = MyClass(logger)
obj.my_function()