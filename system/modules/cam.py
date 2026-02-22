import asyncio
import random
import time
from collections import deque
from .history import History

class Camera(History):
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

    
    class_map = {
        0: 'car',
        1: 'van',
        2: 'truck',
        3: 'pedestrian',
        4: 'person_sitting',
        5: 'cyclist',
        6: 'tram',
        7: 'misc',
    }

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup(**kwargs)
        
    def setup(self, **kwargs):
        # Implement Setup here
        pass
        
    async def read(self):
        # 1. Read the sensor values and run ai model (placeholder)
        value = [
            {'class': 2, 'bbox': [10, 20, 40, 30], 'name': 'car'},
            {'class': 4, 'bbox': [60, 30, 90, 40], 'name': 'truck'},
        ]

        # 2. Record the action in history
        self.save_history(value)