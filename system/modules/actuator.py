import asyncio
import random
import time
from collections import deque

class Actuator:
    """Linear actuator, functions in extend as True/False only"""
    
    def __init__(self, max_history=10, **kwargs):
        super().__init__(**kwargs)

        self.history = {}
        self.history['values'] = deque(maxlen=max_history)
        self.history['ticks'] = deque(maxlen=max_history)
        self.setup(**kwargs)
        
    def setup(**kwargs):
        # Implement Setup here
        pass
        
    async def write(self, value: bool):
        # 1. Perform the physical action (placeholder)
        
        # 2. Record the action in history
        self.history['values'].append(value)
        self.history['ticks'].append(time.time())
