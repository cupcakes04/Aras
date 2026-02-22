import numpy as np
import time
from typing import List, Dict, Optional, Tuple
from scipy.optimize import linear_sum_assignment


class KalmanFilter:
    """
    Simple 2D Kalman filter for tracking object position and velocity.
    State: [x, y, vx, vy]
    """
    
    def __init__(self, dt: float = 0.05):
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
        q = 0.5  # process noise magnitude
        self.Q = q * np.array([
            [dt**4/4, 0, dt**3/2, 0],
            [0, dt**4/4, 0, dt**3/2],
            [dt**3/2, 0, dt**2, 0],
            [0, dt**3/2, 0, dt**2]
        ])
        
        # Measurement noise covariance
        r = 0.3  # measurement noise (metres)
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
    
    def __init__(self, detection: Dict, dt: float = 0.05):
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
        self.kf.x = np.array([
            detection['world_x'],
            detection['world_y'],
            0.0,  # initial velocity unknown
            0.0
        ])
        
        self.name = detection['name']
        self.cam_only = detection['cam_only']
        self.confidence = detection['confidence']
        
        # Lifecycle management
        self.hits = 1  # number of consecutive detections
        self.misses = 0  # number of consecutive misses
        self.age = 0  # total frames since creation
        self.last_update = time.time()
        
        # Track state
        self.state = 'tentative'  # tentative -> confirmed -> deleted
        
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
        z = np.array([detection['world_x'], detection['world_y']])
        self.kf.update(z)
        
        # Update attributes
        self.name = detection['name']
        self.cam_only = detection['cam_only']
        self.confidence = detection['confidence']
        
        # Update lifecycle
        self.hits += 1
        self.misses = 0
        self.last_update = time.time()
        
        # Promote to confirmed if enough hits
        if self.state == 'tentative' and self.hits >= 3:
            self.state = 'confirmed'
            
    def mark_missed(self):
        """Mark track as missed this frame."""
        self.misses += 1
        self.confidence *= 0.9  # decay confidence
        
        # Mark for deletion if too many misses
        if self.misses > 5:
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
            'hits': self.hits,
            'misses': self.misses,
            'age': self.age,
            'state': self.state,
        }


class TrackManager:
    """
    Multi-object tracker with Hungarian algorithm for detection-to-track association.
    """
    
    def __init__(self, dt: float = 0.05, max_distance: float = 2.0):
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
            for j, det in enumerate(detections):
                dx = tx - det['world_x']
                dy = ty - det['world_y']
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
