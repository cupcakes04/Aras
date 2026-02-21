import asyncio
import random
import time
from collections import deque

class Camera:
    """
    Camera
    - functions in reading frames 
    - data of 

    
    Example output from a library function like radar.get_targets(), each id can have multiple dicts
    ```python
        [
            {'class': 2, 'bbox': [x1,y1,x2,y2], 'name': 'car'},
            {'class': 4, 'bbox': [x1,y1,x2,y2], 'name': 'truck'},
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
        # 1. Read the sensor values and run ai model (placeholder)
        value = [
            {'class': 2, 'bbox': [10, 20, 40, 30], 'name': 'car'},
            {'class': 4, 'bbox': [60, 30, 90, 40], 'name': 'truck'},
        ]

        # 2. Record the action in history
        self.history['values'].append(value)
        self.history['ticks'].append(time.time())