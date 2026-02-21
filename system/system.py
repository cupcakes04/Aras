import asyncio
from .modules import Camera, IMU, Radar, Actuator, Vibrator, Speaker
from .BEV import BEV

class System:

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Sensors
        self.camera   = Camera(max_history=10)
        self.imu      = IMU(max_history=10)
        self.radar    = Radar(max_history=10)
        self.actuator = Actuator(max_history=10)
        self.vibrator1 = Vibrator(max_history=10)
        self.vibrator2 = Vibrator(max_history=10)
        self.speaker  = Speaker(max_history=10)

        # Configure radar and cam to Bird's eye view (top down)
        self.bev = BEV(
            camera=self.camera,
            imu=self.imu,
            radar=self.radar,

            # Replace with calibrated values
            image_pts=[[120, 80], [520, 80], [520, 400], [120, 400]],
            world_pts=[[-1.5, 2.5], [1.5, 2.5], [1.5, -2.5], [-1.5, -2.5]],
            camera_matrix=[[1, 0, 1],   
                           [0, 1, 1],
                           [0, 0, 1]],
        )

        # Initialise outputs
        self._actuator_cmd: bool  = True
        self._vibrator1_cmd: float = 0.0
        self._vibrator2_cmd: float = 0.0
        self._speaker_cmd: tuple[str | None, float] = (None, 0.0)


    # |-------------------------------------------------------------|
    # |----------------------- Core Logic --------------------------|
    # |-------------------------------------------------------------|

    def collision_detector(self):
        """
        Fuse camera and radar detections, filter by distance resolution,
        and detect front/side collision threats.
        """
        results = self.bev.get_world_points()
        camera_dets = results['camera']
        radar_targets = results['radar']

        # Filter radar targets by dist_res (0 means no valid detection)
        valid_radar = [t for t in radar_targets if t.get('dist_res', 0) > 0]

        # Merge camera and radar detections (simple spatial matching)
        # For each radar target, find closest camera detection within threshold
        fused_objects = []
        match_threshold = 1.0  # metres

        for radar in valid_radar:
            rx, ry = radar['world_x'], radar['world_y']
            best_match = None
            best_dist = match_threshold

            for cam in camera_dets:
                cx, cy = cam['world_x'], cam['world_y']
                dist = ((rx - cx)**2 + (ry - cy)**2)**0.5
                if dist < best_dist:
                    best_dist = dist
                    best_match = cam

            # Fuse radar + camera if matched, else radar-only
            obj = {
                'x': rx,
                'y': ry,
                'speed': radar.get('speed', 0),
                'class': best_match.get('class') if best_match else None,
                'name': best_match.get('name') if best_match else 'unknown',
            }
            fused_objects.append(obj)

        # Collision zones (bike-centric frame: +y forward, +x right)
        # Front: y > 0.5, |x| < 1.0
        # Left:  x < -0.3, y > 0
        # Right: x > 0.3,  y > 0
        front_threat = False
        side_threat_left = False
        side_threat_right = False
        closest_front_dist = float('inf')

        for obj in fused_objects:
            x, y = obj['x'], obj['y']
            dist = (x**2 + y**2)**0.5

            # Front zone
            if y > 0.5 and abs(x) < 1.0:
                front_threat = True
                closest_front_dist = min(closest_front_dist, dist)

            # Left side
            if x < -0.3 and y > 0 and dist < 2.0:
                side_threat_left = True

            # Right side
            if x > 0.3 and y > 0 and dist < 2.0:
                side_threat_right = True

        # Update actuator command (extend if front threat detected)
        self._actuator_cmd = front_threat

        # Update vibrator intensity based on proximity
        if front_threat and closest_front_dist < float('inf'):
            # Closer = stronger vibration (inverse distance, clamped 0-1)
            intensity = max(0.0, min(1.0, 3.0 / closest_front_dist - 0.5))
            self._vibrator_cmd = intensity
        
        # mild side warning
        elif side_threat_left:
            self._vibrator1_cmd = 0.3
        elif side_threat_right:
            self._vibrator2_cmd = 0.3
        else:
            self._vibrator_cmd = 0.0


    # |-------------------------------------------------------------|
    # |----------------------- Run system --------------------------|
    # |-------------------------------------------------------------|

    @staticmethod
    def assign_task(coro_func, delay):
        """Wrap a coroutine function in a periodic loop."""
        async def periodic():
            while True:
                await coro_func()
                await asyncio.sleep(delay)
        return asyncio.create_task(periodic())

    async def run(self):
        """Spawn all sensor, actuator and BEV tasks then run indefinitely."""
        tasks = [
            # Inputs
            self.assign_task(self.camera.read, delay=0.025),    # 40 Hz
            self.assign_task(self.imu.read, delay=0.01),        # 100 Hz
            self.assign_task(self.radar.read, delay=0.05),      # 20 Hz

            # Processing
            self.assign_task(self.collision_detector, delay=0.05),  # 20 Hz

            # Outputs (lambda wrappers to pass current command values)
            self.assign_task(lambda: self.actuator.write(self._actuator_cmd), delay=0.05),
            self.assign_task(lambda: self.vibrator1.write(self._vibrator1_cmd), delay=0.05),
            self.assign_task(lambda: self.vibrator2.write(self._vibrator2_cmd), delay=0.05),
            self.assign_task(lambda: self.speaker.write(self._speaker_cmd), delay=0.05),
        ]
        
        await asyncio.gather(*tasks)
        
if __name__ == "__main__":
    system = System()
    asyncio.run(system.run())
