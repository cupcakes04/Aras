import asyncio
import random
import time
from collections import deque
from .history import History

class Actuator(History):
    """Linear actuator, functions in extend as True/False only"""
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup(**kwargs)
        
    def setup(self, **kwargs):
        # Implement Setup here
        pass
        
    async def write(self, value: bool):
        # 1. Perform the physical action (placeholder)
        
        # 2. Record the action in history
        self.save_history(value)
