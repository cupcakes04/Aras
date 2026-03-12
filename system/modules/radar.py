import asyncio
import random
import time
from collections import deque
from .history import History

class Radar(History):
    """
    Radar
    - functions in reading every detected objects 
    - data of 
        - angle  : degrees, -90 (left) to +90 (right)
        - distance: metres
        - direction: True = approaching, False = receding
        - speed  : km/h (relative to radar/bike)
        - snr    : 0.0-1.0 normalised signal-to-noise ratio (confidence)
    
    Example output:
    ```python
        [
            {'angle': 30,  'distance': 10, 'direction': True,  'speed': 50, 'snr': 0.6},
            {'angle': -15, 'distance': 5,  'direction': False, 'speed': 10, 'snr': 0.9},
        ]
    ```
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setup(**kwargs)
        
    def setup(self, **kwargs):
        # Implement Setup here
        pass
        
    async def read(self):
        # 1. Read the sensor values (placeholder)
        value = [
            {'angle': 30,  'distance': 10, 'direction': True,  'speed': 2, 'snr': 0.6},
            {'angle': -15, 'distance': 5,  'direction': False, 'speed': 1, 'snr': 0.9},
        ]

        # 2. Record the action in history
        self.save_history(value)