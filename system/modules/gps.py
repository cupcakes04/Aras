import asyncio
import random
import time
from collections import deque
from .history import History

class GPS(History):
    """
    GPS
    - functions in reafing satelite data
    
    Example output from a library function like radar.get_targets(), each id can have multiple dicts
    ```python
        {
            "latitude": 3.1390, [-90,90]
            "longitude": 101.6869, [-180,180]
            "speed_kmh": 44.6 kmh
        }

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
        value = {
            "latitude": 3.1390,
            "longitude": 101.6869,
            "speed_kmh": 44.6
        }

        # 2. Record the action in history
        self.save_history(value)