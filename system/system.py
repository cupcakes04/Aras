import asyncio
import math
from modules import Camera, IMU, Radar, Actuator, Vibrator, Speaker
from BEV import BEV
from tracker import TrackManager
from app.visualisation import Visualisation

# https://github.com/cupcakes04/Aras

class System:

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Sensors
        self.camera = Camera(max_history=10)
        self.imu = IMU(max_history=10)
        self.radar_front = Radar(max_history=10)
        self.radar_back = Radar(max_history=10)
        self.actuator = Actuator(max_history=10)
        self.vibrator_left = Vibrator(max_history=10)
        self.vibrator_right = Vibrator(max_history=10)
        self.speaker = Speaker(max_history=10)

        # Configure radar and cam to Bird's eye view (top down)
        self.bev = BEV(
            camera=self.camera,
            imu=self.imu,
            radar_front=self.radar_front,
            radar_back=self.radar_back,

            # Replace with calibrated values
            image_pts=[[10, 20], [60, 20], [60, 40], [10, 40]],  # bbox corners
            world_pts=[[0, 2], [0.5, 2], [0.5, 1.5], [0, 1.5]],  # real positions in metres
            
            # Example for 640x480 camera
            camera_matrix=[[1, 0, 1],   # fx, 0, cx (optical center x)
                        [0, 1, 1],   # 0, fy, cy (optical center y)
                        [0, 0, 1]]
        )

        # Initialize tracker
        self.tracker = TrackManager(dt=0.05, max_distance=2.0)

        # Initialize visualization
        self.visualisation = Visualisation(self, port=8765)

        # Initialise outputs
        self._actuator_cmd: bool  = True
        self._vibrator_left_cmd: float = 0.0
        self._vibrator_right_cmd: float = 0.0
        self._speaker_cmd: tuple[str | None, float] = (None, 0.0)
        self.objects: list = []
        self.traffic_signs: list = []

    async def report(self):
        print("\n" + "="*60)
        print("SYSTEM STATUS")
        print("="*60)
        
        # Show tracked objects
        tracks = self.tracker.get_all_tracks()
        confirmed = [t for t in tracks if t['state'] == 'confirmed']
        tentative = [t for t in tracks if t['state'] == 'tentative']
        
        print(f"\nTracked Objects: {len(confirmed)} confirmed, {len(tentative)} tentative")
        for obj in confirmed:
            vx, vy = obj['vx'], obj['vy']
            speed = (vx**2 + vy**2)**0.5
            print(f"  ID {obj['id']:3d}: {obj['name']:12s} @ ({obj['world_x']:+.2f}, {obj['world_y']:+.2f})m "
                  f"v={speed:4.1f}m/s conf={obj['confidence']:.2f} "
                  f"hits={obj['hits']:2d} {'[CAM]' if obj['cam_only'] else '[RAD]'}")
        
        if tentative:
            print(f"\n  Tentative ({len(tentative)}):")
            for obj in tentative:
                print(f"    ID {obj['id']:3d}: {obj['name']:12s} @ ({obj['world_x']:+.2f}, {obj['world_y']:+.2f})m "
                      f"hits={obj['hits']}")
        
        # Show commands
        print(f"\nCommands:")
        print(f"  Actuator:      {self._actuator_cmd}")
        print(f"  Vibrator Left: {self._vibrator_left_cmd:.2f}")
        print(f"  Vibrator Right:{self._vibrator_right_cmd:.2f}")
        print(f"  Speaker:       {self._speaker_cmd}")
        print("="*60)


    # |-------------------------------------------------------------|
    # |----------------------- Core Logic --------------------------|
    # |-------------------------------------------------------------|

    async def generate_world_objects(self):
        """
        Generate unified object list from camera and radar with two-level fusion:
        1. Radar-confirmed objects (high confidence)
        2. Camera-only objects (lower confidence)
        
        Also processes traffic signs separately (camera-only, no radar fusion).
        
        Updates:
        - self.objects: list of tracked objects (cars, pedestrians, etc.)
        - self.traffic_signs: list of traffic sign detections (camera-only)
        """
        results = self.bev.get_world_points()
        camera_results = results['camera']
        camera_objs = camera_results.get('objs', [])
        camera_signs = camera_results.get('signs', [])
        radar_front = results['radar_front']
        radar_back = results['radar_back']
        
        # Combine front and back radar targets
        radar_targets = radar_front + radar_back

        objects = []
        obj_id = 0
        matched_cam_indices = set()

        # Level 1: Radar-confirmed objects (merge with camera if available)
        # Group radar targets by distance resolution cells to avoid duplicates
        # Use 2D gating: range resolution (radial) + spatial threshold (lateral)
        radar_groups = []
        for radar in radar_targets:
            dist_res = radar.get('dist_res', 0)
            if dist_res <= 0:
                continue  # Invalid detection
            
            # Check if this radar target is within resolution cell of any existing group
            rx, ry = radar['world_x'], radar['world_y']
            r_dist = (rx**2 + ry**2)**0.5
            
            merged = False
            for group in radar_groups:
                gx, gy = group['world_x'], group['world_y']
                g_dist = (gx**2 + gy**2)**0.5
                
                range_gate = dist_res / 1000.0  # radial resolution in metres
                spatial_gate = 1.0  # lateral separation threshold in metres
                
                # 2D gating: range difference AND spatial distance
                range_diff = abs(r_dist - g_dist)
                euclidean_dist = ((rx - gx)**2 + (ry - gy)**2)**0.5
                
                if range_diff < range_gate and euclidean_dist < spatial_gate:
                    # Keep the one with better dist_res (smaller = better)
                    if dist_res < group.get('dist_res', float('inf')):
                        group.update(radar)
                    merged = True
                    break
            
            if not merged:
                radar_groups.append(radar.copy())

        # Now fuse each radar group with closest camera detection
        for radar in radar_groups:
            rx, ry = radar['world_x'], radar['world_y']
            dist_res_m = radar['dist_res'] / 1000.0  # mm to m
            
            best_match = None
            best_match_idx = None
            best_dist = 1.0  # matching threshold in metres

            for idx, cam in enumerate(camera_objs):
                if idx in matched_cam_indices:
                    continue
                cx, cy = cam['world_x'], cam['world_y']
                dist = ((rx - cx)**2 + (ry - cy)**2)**0.5
                if dist < best_dist:
                    best_dist = dist
                    best_match = cam
                    best_match_idx = idx

            # Confidence as relative quality weight from dist_res
            # Better (smaller) dist_res = higher weight
            # Normalize to typical range: 50mm (excellent) to 500mm (poor)
            # Use logarithmic scale to avoid artificial bounds
            dist_res_mm = radar['dist_res']
            if dist_res_mm < 50:
                quality = 1.0
            elif dist_res_mm > 500:
                quality = 0.2
            else:
                # Logarithmic interpolation
                quality = 1.0 - 0.8 * (math.log(dist_res_mm / 50) / math.log(10))
            
            # Boost confidence if matched with camera
            confidence = quality * 1.2 if best_match else quality
            confidence = min(1.0, confidence)  # clamp to 1.0

            obj = {
                'id': obj_id,
                'name': best_match.get('name', 'unknown') if best_match else 'unknown',
                'world_x': rx,
                'world_y': ry,
                'confidence': confidence,
                'cam_only': False,
            }
            objects.append(obj)
            obj_id += 1
            
            if best_match_idx is not None:
                matched_cam_indices.add(best_match_idx)

        # Level 2: Camera-only objects (no radar confirmation)
        for idx, cam in enumerate(camera_objs):
            if idx in matched_cam_indices:
                continue  # Already matched with radar
            
            obj = {
                'id': obj_id,
                'name': cam.get('name', 'unknown'),
                'world_x': cam['world_x'],
                'world_y': cam['world_y'],
                'confidence': 0.4,  # Lower confidence without radar confirmation
                'cam_only': True,
            }
            objects.append(obj)
            obj_id += 1

        self.objects = objects
        
        # Process traffic signs separately (camera-only, no radar fusion)
        signs = []
        sign_id = 0
        for sign in camera_signs:
            sign_obj = {
                'id': sign_id,
                'name': sign.get('name', 'unknown_sign'),
                'world_x': sign['world_x'],
                'world_y': sign['world_y'],
                'confidence': 0.7,  # Fixed confidence for signs
            }
            signs.append(sign_obj)
            sign_id += 1
        
        self.traffic_signs = signs

    async def collision_detector(self):
        """
        Detect front/side collision threats using tracked objects.
        """
        # Get raw detections and update tracker
        tracked_objects = self.tracker.update(self.objects)

        # Collision zones (bike-centric frame: +y forward, +x right)
        # Front: y > 0.5, |x| < 1.0
        # Left:  x < -0.3, y > 0
        # Right: x > 0.3,  y > 0
        front_threat = False
        side_threat_left = False
        side_threat_right = False
        closest_front_dist = float('inf')
        front_confidence = 0.0

        for obj in tracked_objects:
            x, y = obj['world_x'], obj['world_y']
            dist = (x**2 + y**2)**0.5
            conf = obj['confidence']

            # Only consider high-confidence detections for collision
            if conf < 0.3:
                continue

            # Front zone
            if y > 0.5 and abs(x) < 1.0:
                front_threat = True
                if dist < closest_front_dist:
                    closest_front_dist = dist
                    front_confidence = conf

            # Left side
            if x < -0.3 and y > 0 and dist < 2.0:
                side_threat_left = True

            # Right side
            if x > 0.3 and y > 0 and dist < 2.0:
                side_threat_right = True

        # Update actuator command (extend if front threat detected)
        self._actuator_cmd = front_threat

        # Update vibrator intensity based on proximity and confidence
        if front_threat and closest_front_dist < float('inf'):
            # Closer = stronger vibration, weighted by confidence
            base_intensity = max(0.0, min(1.0, 3.0 / closest_front_dist - 0.5))
            intensity = base_intensity * front_confidence
            self._vibrator_left_cmd = intensity
            self._vibrator_right_cmd = intensity
        elif side_threat_left:
            self._vibrator_left_cmd = 0.3
            self._vibrator_right_cmd = 0.0
        elif side_threat_right:
            self._vibrator_left_cmd = 0.0
            self._vibrator_right_cmd = 0.3
        else:
            self._vibrator_left_cmd = 0.0
            self._vibrator_right_cmd = 0.0


    # |-------------------------------------------------------------|
    # |----------------------- Run system --------------------------|
    # |-------------------------------------------------------------|


    @staticmethod
    def assign_task(coro_func, period, **kwargs):
        async def periodic():
            while True:
                start = asyncio.get_event_loop().time()
                await coro_func(**kwargs)
                elapsed = asyncio.get_event_loop().time() - start
                await asyncio.sleep(max(0, period - elapsed))
        return asyncio.create_task(periodic())

    async def run(self):
        """Spawn all sensor, actuator and BEV tasks then run indefinitely."""
        tasks = [
            asyncio.create_task(system.visualisation.start(period=0.1)),

            # Inputs
            self.assign_task(self.camera.read, period=0.025),
            self.assign_task(self.imu.read, period=0.01),
            self.assign_task(self.radar_front.read, period=0.05),
            self.assign_task(self.radar_back.read, period=0.05),

            # Processing
            self.assign_task(self.generate_world_objects, period=0.05),
            self.assign_task(self.collision_detector, period=0.05),
            self.assign_task(self.report, period=1.0),

            # Outputs (lambda wrappers to pass current command values)
            self.assign_task(self.actuator.write, value=self._actuator_cmd, period=0.05),
            self.assign_task(self.vibrator_left.write, value=self._vibrator_left_cmd, period=0.05),
            self.assign_task(self.vibrator_right.write, value=self._vibrator_right_cmd, period=0.05),
            self.assign_task(self.speaker.write, value=self._speaker_cmd, period=0.05),
        ]
        await asyncio.gather(*tasks)
        
if __name__ == "__main__":
    system = System()
    asyncio.run(system.run())
