from functools import wraps
from time import time


def rate_limit(frequency: float):
    """
    Decorator that limits the frequency at which a function can be called.
    
    Args:
        frequency: Maximum calls per second. If frequency=1, only 1 call per second is allowed.
        Any subsequent call after the first call will just return None for the next second.
        Useful for limiting operation (such as image processing) at a fixed framerate, independent
        of the image publishing frequency
    """
    min_interval = 1.0 / frequency
    
    def decorator(func):
        func.last_called = 0.0
        
        @wraps(func)
        def wrapper(*args, **kwargs):
            now = time()
            if now - func.last_called < min_interval:
                return None
            func.last_called = now
            return func(*args, **kwargs)
        
        return wrapper
    
    return decorator