import asyncio
import random
import time
from collections import deque
from .history import History

class Speaker(History):
    """Speaker, 
    data in:
        - str of characters (not sure, maybe a list of stuff idk)
        - 0.0~1.0 signal strength, normalised
    """
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup(**kwargs)
        
    def setup(self, **kwargs):
        # Implement Setup here
        pass
        
    async def write(self, value: str):
        # 1. Perform the physical action (placeholder)
        
        # 2. Record the action in history
        self.save_history(value)
