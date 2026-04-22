import numpy as np
import time
from typing import List, Dict, Optional, Tuple
from scipy.optimize import linear_sum_assignment

# ==============================================================================
# GLOBAL TUNING PARAMETERS
# ==============================================================================
KALMAN_DT = 0.05                 # 0.05 Time step in seconds (e.g., 0.05 = 20Hz)
KALMAN_PROCESS_NOISE_Q = 0.5     # 0.5 Process noise magnitude (acceleration variance)
KALMAN_MEASURE_NOISE_R = 0.3     # 0.3 Measurement noise covariance in metres
RADAR_VELOCITY_ALPHA = 0.3       # 0.3 Alpha filter weight for blending radar speed (0.0 to 1.0)
TRACK_CONFIRM_HITS = 3           # 3 Number of consecutive hits to confirm a track
TRACK_DELETE_MISSES = 5          # 5 Number of consecutive misses to delete a track
TRACK_CONFIDENCE_DECAY = 0.9     # 0.9 Multiplier to decay confidence on a missed frame
TRACK_MAX_DISTANCE = 2.0         # 2 Maximum Euclidean distance (m) for Hungarian assignment
# ==============================================================================


class KalmanFilter:
    """
    Simple 2D Kalman filter for tracking object position and velocity.
    State: [x, y, vx, vy]
    """
    
    def __init__(self, dt: float = KALMAN_DT):
        """
        Parameters
        ----------
        dt : float
            Time step in seconds (default 0.05 = 20Hz)
        """
        self.dt = dt
        
        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Measurement matrix (observe position only)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Process noise covariance
        q = KALMAN_PROCESS_NOISE_Q  # process noise magnitude
        self.Q = q * np.array([
            [dt**4/4, 0, dt**3/2, 0],
            [0, dt**4/4, 0, dt**3/2],
            [dt**3/2, 0, dt**2, 0],
            [0, dt**3/2, 0, dt**2]
        ])
        
        # Measurement noise covariance
        r = KALMAN_MEASURE_NOISE_R  # measurement noise (metres)
        self.R = r * np.eye(2)
        
        # State estimate and covariance
        self.x = np.zeros(4)  # [x, y, vx, vy]
        self.P = np.eye(4) * 10.0  # initial uncertainty
        
    def predict(self):
        """Predict next state."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, z: np.ndarray):
        """
        Update state with measurement.
        
        Parameters
        ----------
        z : np.ndarray
            Measurement [x, y]
        """
        y = z - self.H @ self.x  # innovation
        S = self.H @ self.P @ self.H.T + self.R  # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        
    def get_state(self) -> Tuple[float, float, float, float]:
        """Return current state (x, y, vx, vy)."""
        return tuple(self.x)


class Track:
    """
    Single object track with Kalman filter and lifecycle management.
    """
    
    _next_id = 0
    
    def __init__(self, detection: Dict, dt: float = KALMAN_DT):
        """
        Parameters
        ----------
        detection : dict
            Initial detection with 'world_x', 'world_y', 'confidence', 'name', 'cam_only'
        dt : float
            Time step for Kalman filter
        """
        self.id = Track._next_id
        Track._next_id += 1
        
        self.kf = KalmanFilter(dt=dt)
        radar_speed = detection.get('radar_speed')
        radar_direction = detection.get('radar_direction')
        vy_init = self._radar_vy(detection['world_y'], radar_speed, radar_direction)
        self.kf.x = np.array([
            detection['world_x'],
            detection['world_y'],
            0.0,
            vy_init,
        ])
        
        self.name = detection['name']
        self.cam_only = detection['cam_only']
        self.confidence = detection['confidence']
        self.radar_speed = radar_speed
        self.radar_direction = radar_direction
        
        # Lifecycle management
        self.hits = 1  # number of consecutive detections
        self.misses = 0  # number of consecutive misses
        self.age = 0  # total frames since creation
        self.last_update = time.time()
        
        # Track state
        self.state = 'tentative'  # tentative -> confirmed -> deleted
        
    @staticmethod
    def _radar_vy(world_y: float, radar_speed, radar_direction) -> float:
        """
        Convert radar speed (km/h) + direction into an initial vy estimate (m/s).
        direction=True (approaching): object closes on bike, so vy opposes its y-position.
        direction=False (receding):   object moves away, so vy follows its y-position sign.
        Returns 0.0 if radar data is unavailable.
        """
        if radar_speed is None or radar_speed <= 0:
            return 0.0
        speed_ms = radar_speed / 3.6  # km/h -> m/s
        y_sign = 1.0 if world_y >= 0 else -1.0
        if radar_direction is False: # TEMPORARILY INVERTED True/False here!
            return -y_sign * speed_ms  # approaching: closing velocity
        else:
            return y_sign * speed_ms   # receding: opening velocity

    def predict(self):
        """Predict next state."""
        self.kf.predict()
        self.age += 1
        
    def update(self, detection: Dict):
        """
        Update track with new detection.
        
        Parameters
        ----------
        detection : dict
            Detection with 'world_x', 'world_y', 'confidence', 'name', 'cam_only'
        """
        delay = detection.get('cam_delay', 0.0) if detection.get('cam_only') else 0.0
        vx, vy = self.kf.x[2], self.kf.x[3]
        
        z = np.array([
            detection['world_x'] + vx * delay,
            detection['world_y'] + vy * delay
        ])
        self.kf.update(z)
        
        # Update attributes
        self.name = detection['name']
        self.cam_only = detection['cam_only']
        self.confidence = detection['confidence']
        self.radar_speed = detection.get('radar_speed')
        self.radar_direction = detection.get('radar_direction')

        # Nudge Kalman vy toward radar-derived velocity when radar data is present
        vy_radar = self._radar_vy(detection['world_y'], self.radar_speed, self.radar_direction)
        if vy_radar != 0.0:
            alpha = RADAR_VELOCITY_ALPHA  # blend weight: 30% radar hint, 70% filter estimate
            self.kf.x[3] = (1 - alpha) * self.kf.x[3] + alpha * vy_radar
        
        # Update lifecycle
        self.hits += 1
        self.misses = 0
        self.last_update = time.time()
        
        # Promote to confirmed if enough hits
        if self.state == 'tentative' and self.hits >= TRACK_CONFIRM_HITS:
            self.state = 'confirmed'
            
    def mark_missed(self):
        """Mark track as missed this frame."""
        self.misses += 1
        self.confidence *= TRACK_CONFIDENCE_DECAY  # decay confidence
        
        # Mark for deletion if too many misses
        if self.misses > TRACK_DELETE_MISSES:
            self.state = 'deleted'
            
    def get_position(self) -> Tuple[float, float]:
        """Return current position (x, y)."""
        x, y, _, _ = self.kf.get_state()
        return x, y
        
    def get_velocity(self) -> Tuple[float, float]:
        """Return current velocity (vx, vy)."""
        _, _, vx, vy = self.kf.get_state()
        return vx, vy
        
    def to_dict(self) -> Dict:
        """Convert track to dict format."""
        x, y, vx, vy = self.kf.get_state()
        return {
            'id': self.id,
            'name': self.name,
            'world_x': float(x),
            'world_y': float(y),
            'vx': float(vx),
            'vy': float(vy),
            'confidence': self.confidence,
            'cam_only': self.cam_only,
            'radar_speed': self.radar_speed,
            'radar_direction': self.radar_direction,
            'hits': self.hits,
            'misses': self.misses,
            'age': self.age,
            'state': self.state,
        }


class TrackManager:
    """
    Multi-object tracker with Hungarian algorithm for detection-to-track association.
    """
    
    def __init__(self, dt: float = KALMAN_DT, max_distance: float = TRACK_MAX_DISTANCE):
        """
        Parameters
        ----------
        dt : float
            Time step for Kalman filters
        max_distance : float
            Maximum distance (metres) for association
        """
        self.dt = dt
        self.max_distance = max_distance
        self.tracks: List[Track] = []
        
    def update(self, detections: List[Dict]) -> List[Dict]:
        """
        Update tracks with new detections.
        
        Parameters
        ----------
        detections : list of dict
            List of detections from generate_world_objects()
            
        Returns
        -------
        list of dict
            List of confirmed tracks
        """
        # Predict all tracks
        for track in self.tracks:
            track.predict()
            
        # Associate detections to tracks
        if len(self.tracks) > 0 and len(detections) > 0:
            matched, unmatched_tracks, unmatched_dets = self._associate(detections)
            
            # Update matched tracks
            for track_idx, det_idx in matched:
                self.tracks[track_idx].update(detections[det_idx])
                
            # Mark unmatched tracks as missed
            for track_idx in unmatched_tracks:
                self.tracks[track_idx].mark_missed()
                
            # Create new tracks for unmatched detections
            for det_idx in unmatched_dets:
                self.tracks.append(Track(detections[det_idx], dt=self.dt))
        elif len(detections) > 0:
            # No existing tracks, create new ones
            for det in detections:
                self.tracks.append(Track(det, dt=self.dt))
        else:
            # No detections, mark all tracks as missed
            for track in self.tracks:
                track.mark_missed()
                
        # Remove deleted tracks
        self.tracks = [t for t in self.tracks if t.state != 'deleted']
        
        # Return confirmed tracks only
        confirmed = [t.to_dict() for t in self.tracks if t.state == 'confirmed']
        return confirmed
        
    def _associate(self, detections: List[Dict]) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """
        Associate detections to tracks using Hungarian algorithm.
        
        Returns
        -------
        matched : list of (track_idx, det_idx)
        unmatched_tracks : list of track_idx
        unmatched_dets : list of det_idx
        """
        # Build cost matrix (Euclidean distance)
        n_tracks = len(self.tracks)
        n_dets = len(detections)
        
        cost_matrix = np.zeros((n_tracks, n_dets))
        for i, track in enumerate(self.tracks):
            tx, ty = track.get_position()
            vx, vy = track.kf.x[2], track.kf.x[3]
            for j, det in enumerate(detections):
                delay = det.get('cam_delay', 0.0) if det.get('cam_only') else 0.0
                dx = tx - (det['world_x'] + vx * delay)
                dy = ty - (det['world_y'] + vy * delay)
                dist = np.sqrt(dx**2 + dy**2)
                cost_matrix[i, j] = dist
                
        # Apply Hungarian algorithm
        track_indices, det_indices = linear_sum_assignment(cost_matrix)
        
        # Filter out matches that exceed max_distance
        matched = []
        for t_idx, d_idx in zip(track_indices, det_indices):
            if cost_matrix[t_idx, d_idx] <= self.max_distance:
                matched.append((t_idx, d_idx))
                
        # Find unmatched tracks and detections
        matched_track_indices = {t_idx for t_idx, _ in matched}
        matched_det_indices = {d_idx for _, d_idx in matched}
        
        unmatched_tracks = [i for i in range(n_tracks) if i not in matched_track_indices]
        unmatched_dets = [i for i in range(n_dets) if i not in matched_det_indices]
        
        return matched, unmatched_tracks, unmatched_dets
        
    def get_all_tracks(self) -> List[Dict]:
        """Return all tracks (including tentative)."""
        return [t.to_dict() for t in self.tracks]
