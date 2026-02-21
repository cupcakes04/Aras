import asyncio
import random
import time
from collections import deque

class Radar:
    """
    Radar
    - functions in reading every detected objects 
    - data of 
        - (x,y)
        - speed and 
        - distance resolution
    
    Example output from a library function like radar.get_targets(), each id can have multiple dicts
    ```python
        [
            {'id': 1, 'x': -150, 'y': 1200, 'speed': 25, 'dist_res': 360}, 
            {'id': 2, 'x': 500,  'y': 3000, 'speed': 0,  'dist_res': 360},
            {'id': 3, 'x': 0,    'y': 0,    'speed': 0,  'dist_res': 0} # 0 means no person
        ]
    ```
    """

    def __init__(self, max_history=10, **kwargs):
        super().__init__(**kwargs)

        self.history = {}
        self.history['values'] = deque(maxlen=max_history)
        self.history['ticks'] = deque(maxlen=max_history)
        self.setup(**kwargs)
        
    def setup(**kwargs):
        # Implement Setup here
        pass
        
    async def read(self):
        # 1. Read the sensor values (placeholder)
        value = [
            {'id': 1, 'x': -150, 'y': 1200, 'speed': 25, 'dist_res': 360}, 
            {'id': 2, 'x': 500,  'y': 3000, 'speed': 0,  'dist_res': 360},
            {'id': 3, 'x': 0,    'y': 0,    'speed': 0,  'dist_res': 0} # 0 means no person
        ]

        # 2. Record the action in history
        self.history['values'].append(value)
        self.history['ticks'].append(time.time())