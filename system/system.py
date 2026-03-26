import asyncio
from modules import Camera, IMU, Radar, Actuator, Vibrator, Speaker, GPS, CameraOld, RadarOld, ActuatorOld, VibratorOld, SpeakerOld, GPSOld
from BEV import BEV
from tracker import TrackManager
from app.visualisation import Visualisation
import yaml
import os

# https://github.com/cupcakes04/Aras


class System:

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # Debug use
        self.camera = CameraOld(max_history=10, config=self.config)
        self.imu = IMU(max_history=10)
        self.radar_front = RadarOld(max_history=10)
        self.radar_back = RadarOld(max_history=10)
        self.gps = GPSOld(max_history=10)
        self.actuator = ActuatorOld(max_history=10)
        self.vibrator_left = VibratorOld(max_history=10)
        self.vibrator_right = VibratorOld(max_history=10)
        self.speaker = Speaker(max_history=10)

        # # Sensors
        # self.camera = Camera(max_history=10, config=self.config)
        # self.imu = IMU(max_history=10)
        # self.radar_front = HardwareRadar(port="/dev/ttyS3", baudrate=115200, max_history=10)
        # self.radar_back = HardwareRadar(port="/dev/ttyS5", baudrate=115200, max_history=10)
        # self.gps = HardwareGPS(port="/dev/ttyS4", baudrate=9600, max_history=10)

        # # Hardware Actuators (with fallback built-in inside the modules)
        # self.actuator = LinearActuator(rpwm_chip=0, rpwm_channel=0, lpwm_chip=0, lpwm_channel=1, max_history=10)
        # self.vibrator_left = HapticVibrator(gpio_chip='/dev/gpiochip0', gpio_line=10, max_history=10)
        # self.vibrator_right = HapticVibrator(gpio_chip='/dev/gpiochip0', gpio_line=11, max_history=10)
        # self.speaker = HardwareSpeaker(max_history=10)

        # Configure radar and cam to Bird's eye view (top down)

        self.bev = BEV(
            camera=self.camera,
            imu=self.imu,
            radar_front=self.radar_front,
            radar_back=self.radar_back,

            # Replace with calibrated values
            image_pts=self.config['bev']['image_pts'],
            world_pts=self.config['bev']['world_pts'],
            
            # Example for 640x480 camera
            camera_matrix=self.config['bev']['camera_matrix'],
        )

        # Initialize tracker
        self.tracker = TrackManager(dt=0.05, max_distance=2.0)

        # Initialize visualization
        self.visualisation = Visualisation(self, port=8765)

        # Initialise outputs
        self._actuator_cmd: bool  = False
        self._vibrator_left_cmd: float = 0.0
        self._vibrator_right_cmd: float = 0.0
        self._speaker_cmd: str | None = None
        self.objects: list = []
        self.traffic_signs: list = []
        
        # Collision detection state
        self.collision_state = {
            'front_threat': False,
            'side_threat_left': False,
            'side_threat_right': False,
            'closest_front_dist': float('inf'),
            'front_confidence': 0.0,
            'vibrator_left': 0.0,
            'vibrator_right': 0.0
        }

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
        print(f"  Speaker:       {self._speaker_cmd or 'None'}")
        print("="*60)


    # |-------------------------------------------------------------|
    # |----------------------- Core Logic --------------------------|
    # |-------------------------------------------------------------|

    async def generate_world_objects(self):
        """
        Generate unified object list from camera and radar with two-level fusion:
        1. Radar-confirmed objects (high confidence, snr used directly as confidence)
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

        # Level 1: Radar-confirmed objects (fuse with closest camera detection)
        for radar in radar_targets:
            snr = radar.get('snr', 0.0)
            if snr <= 0.0:
                continue  # Invalid detection

            rx, ry = radar['world_x'], radar['world_y']

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

            # Boost confidence if matched with camera
            confidence = min(1.0, snr * 1.2) if best_match else snr

            obj = {
                'id': obj_id,
                'name': best_match.get('name', 'unknown') if best_match else 'unknown',
                'world_x': rx,
                'world_y': ry,
                'confidence': confidence,
                'cam_only': False,
                'radar_speed': radar['speed'],
                'radar_direction': radar['direction'],
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
                'radar_speed': None,
                'radar_direction': None,
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
        Detect collision threats using tracked objects and TTC (Time-To-Collision).

        Front lane  : parallel corridor ±LANE_HALF_WIDTH metres either side of bike.
                      Object speed = Kalman vy (m/s, seeded from radar when available).
                      TTC = y / |vy|.  If TTC < TTC_THRESHOLD → collision alert.
                      Actuators (vibrator + brake actuator) fire on front collision only.

        Back lane   : same corridor behind bike (y < 0, approaching = vy > 0).
                      No actuators — sets a per-object 'back_collision' flag for UI.

        Each tracked object gets two annotation fields written onto its dict:
          'front_collision' : bool
          'back_collision'  : bool
        These are forwarded to the visualisation WebSocket payload.
        """
        LANE_HALF_WIDTH = 1.0   # metres either side of centre line
        TTC_THRESHOLD   = 2.0   # seconds — alert if time-to-collision is below this
        MAX_BRAKE_SPEED_KMH = 50.0  # Do not auto-brake if ego speed > 50 km/h
        MIN_BRAKE_SPEED_KMH = 5.0   # Do not auto-brake if ego speed < 5 km/h

        # Get ego speed from GPS history (default to 0 if unavailable)
        ego_speed_kmh = 0.0
        if self.gps.history.get('values'):
            latest_gps = self.gps.history['values'][-1]
            if latest_gps and 'speed_kmh' in latest_gps:
                ego_speed_kmh = float(latest_gps['speed_kmh'])

        # Get raw detections and update tracker
        tracked_objects = self.tracker.update(self.objects)

        front_collision = False
        back_collision  = False
        best_ttc        = float('inf')
        best_conf       = 0.0

        for obj in tracked_objects:
            x,  y  = obj['world_x'], obj['world_y']
            vx, vy = obj['vx'],      obj['vy']
            conf   = obj['confidence']

            obj['front_collision'] = False
            obj['back_collision']  = False

            if conf < 0.3:
                continue

            # Only objects within the lane corridor (parallel lines, not a cone)
            if abs(x) > LANE_HALF_WIDTH:
                continue

            # ── Front objects (y > 0, approaching means vy < 0) ──────────────
            if y > 0 and vy < 0:
                ttc = y / abs(vy)
                if ttc < TTC_THRESHOLD:
                    obj['front_collision'] = True
                    front_collision = True
                    if ttc < best_ttc:
                        best_ttc  = ttc
                        best_conf = conf

            # ── Back objects (y < 0, approaching means vy > 0) ───────────────
            elif y < 0 and vy > 0:
                ttc = abs(y) / vy
                if ttc < TTC_THRESHOLD:
                    obj['back_collision'] = True
                    back_collision = True

        # ── Actuator outputs (front only) ─────────────────────────────────────
        # Only allow the physical brake actuator to fire if within safe speed bounds
        if MIN_BRAKE_SPEED_KMH <= ego_speed_kmh <= MAX_BRAKE_SPEED_KMH:
            self._actuator_cmd = front_collision
        else:
            self._actuator_cmd = False

        if front_collision:
            # Intensity scales with urgency: lower TTC = higher intensity
            intensity = min(1.0, best_conf * (TTC_THRESHOLD / max(best_ttc, 0.1)))
            self._vibrator_left_cmd  = intensity
            self._vibrator_right_cmd = intensity
            self._speaker_cmd = WARN_FCW
        else:
            self._vibrator_left_cmd  = 0.0
            self._vibrator_right_cmd = 0.0
            self._speaker_cmd = WARN_RCW if back_collision else None

        # Store annotated tracked objects for visualisation broadcast
        self.tracked_objects = tracked_objects


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
            self.assign_task(self.gps.read, period=1.0),

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
