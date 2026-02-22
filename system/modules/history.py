import time
from collections import deque

class History:
    """Set up history"""
    
    def __init__(self, max_history=10):
        self.reset_history(max_history)

    def reset_history(self, max_history=10):
        self.history = {}
        self.history['values'] = deque(maxlen=max_history)
        self.history['ticks'] = deque(maxlen=max_history)
        
    def save_history(self, value):
        self.history['values'].append(value)
        self.history['ticks'].append(time.time())