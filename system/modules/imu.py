import asyncio
import random
import time
from collections import deque
from .history import History

class IMU(History):
    """
    IMU
    - functions in reading frames 
    - data of 
        - Accelerometer (accel) → measures linear acceleration along X, Y, Z axes.
        - Gyroscope (gyro) → measures angular velocity (rotation rate) along X, Y, Z axes.
    
    Example output from a library function like radar.get_targets(), each id can have multiple dicts
    ```python
        {
            "accel": {"x": -0.12, "y": 0.98, "z": 0.05},   # in g (gravity units)
            "gyro":  {"x": 0.01, "y": -0.03, "z": 0.00},   # in deg/s
        }
    ```

    """

    def __init__(self, max_history=10, **kwargs):
        super().__init__(**kwargs)

        self.history = {}
        self.history['values'] = deque(maxlen=max_history)
        self.history['ticks'] = deque(maxlen=max_history)
        self.setup(**kwargs)
        
    def setup(self, **kwargs):
        # Implement Setup here
        pass
        
    async def read(self):
        # 1. Read the sensor values
        # Dummy values — bike assumed perfectly upright (no roll/pitch).
        # Replace with real hardware reads when IMU is connected.
        value = {
            "accel": {"x": 0.0, "y": 0.0, "z": 1.0},   # in g (gravity units) — pure gravity on z, no tilt
            "gyro":  {"x": 0.0, "y": 0.0, "z": 0.0},   # in deg/s — stationary / no rotation
        }

        # 2. Record the action in history
        self.save_history(value)