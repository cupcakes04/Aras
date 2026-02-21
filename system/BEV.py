import asyncio
import math
import numpy as np
import cv2


class BEV:
    """
    Bird's-Eye-View fusion layer.

    Holds references to Camera, IMU and Radar instances and reads directly
    from their .history — no sensor I/O here, nothing blocks.

    Camera config
    -------------
    The user marks four pixel corners of a flat ground rectangle whose real-world
    size is known (width_m x height_m).  A homography is computed once at setup
    and reused every frame.

        image_pts  - four (u, v) pixel corners, shape (4, 2), float32
                     order: top-left, top-right, bottom-right, bottom-left
        world_pts  - corresponding (x, y) metres in the BEV frame, shape (4, 2)
                     e.g. for a 3 m x 5 m rectangle centred at origin:
                     [[-1.5, 2.5], [1.5, 2.5], [1.5, -2.5], [-1.5, -2.5]]

        camera_matrix - (3, 3) intrinsic matrix [[fx,0,cx],[0,fy,cy],[0,0,1]]
                        Obtain this once with BEV.calibrate_camera().

    Radar config
    ------------
    Radar reports (x, y) in millimetres.  scale_mm_to_m converts to metres.

    IMU config
    ----------
    Roll and pitch are derived each frame from the latest accel sample and used to:
      - Camera: warp pixel coords back to a level view before applying H.
      - Radar : rotate the reported beam direction onto the flat ground plane.
    """

    def __init__(self, camera, imu, radar,
                 image_pts: np.ndarray, world_pts: np.ndarray,
                 camera_matrix: np.ndarray,
                 scale_mm_to_m: float = 0.001):
        """
        Parameters
        ----------
        camera       : Camera instance (holds .history)
        imu          : IMU instance    (holds .history)
        radar        : Radar instance  (holds .history)
        image_pts    : (4, 2) float32 - pixel corners of the ground rectangle
        world_pts    : (4, 2) float32 - real-world (x, y) metres for each corner
        camera_matrix: (3, 3) float64 - camera intrinsic matrix
        scale_mm_to_m: radar unit -> metres conversion (default 0.001)
        """
        self._camera = camera
        self._imu    = imu
        self._radar  = radar

        image_pts = np.asarray(image_pts, dtype=np.float32)
        world_pts = np.asarray(world_pts, dtype=np.float32)

        self._H, _ = cv2.findHomography(image_pts, world_pts)
        self._K = np.asarray(camera_matrix, dtype=np.float64)
        self._K_inv = np.linalg.inv(self._K)
        self._scale = scale_mm_to_m

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _imu_tilt(imu_values):
        """
        Extract roll and pitch (degrees) from the latest IMU accel sample.

        Uses gravity vector direction:
          roll  = atan2(ay, az)   - lean left/right
          pitch = atan2(-ax, az)  - tilt forward/back
        """
        if not imu_values:
            return 0.0, 0.0
        accel = imu_values[-1]["accel"]
        ax, ay, az = accel["x"], accel["y"], accel["z"]
        roll_rad  = math.atan2(ay, az)
        pitch_rad = math.atan2(-ax, az)
        return math.degrees(roll_rad), math.degrees(pitch_rad)

    def _tilt_rotation_matrix(self, roll_deg: float, pitch_deg: float) -> np.ndarray:
        """
        3x3 rotation matrix R = Rx(pitch) @ Ry(roll) that maps a tilted camera
        frame back to a level (upright) frame.
        """
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)

        Rx = np.array([[1,          0,           0],
                       [0,  math.cos(p), -math.sin(p)],
                       [0,  math.sin(p),  math.cos(p)]], dtype=np.float64)

        Ry = np.array([[ math.cos(r), 0, math.sin(r)],
                       [          0,  1,           0],
                       [-math.sin(r), 0, math.cos(r)]], dtype=np.float64)

        return Rx @ Ry

    def _tilt_correction_H(self, roll_deg: float, pitch_deg: float) -> np.ndarray:
        """Per-frame homography that un-tilts pixel coordinates: K @ R @ K_inv."""
        R = self._tilt_rotation_matrix(roll_deg, pitch_deg)
        return self._K @ R @ self._K_inv

    def _pixel_to_world(self, u: float, v: float,
                        roll_deg: float, pitch_deg: float) -> np.ndarray:
        """
        1. Un-tilt pixel (u, v) using roll/pitch -> level pixel (u', v')
        2. Apply ground homography H -> world (x, y) metres.
        """
        H_tilt = self._tilt_correction_H(roll_deg, pitch_deg)
        pt = np.array([[[u, v]]], dtype=np.float32)
        pt_level = cv2.perspectiveTransform(pt, H_tilt.astype(np.float32))
        world = cv2.perspectiveTransform(pt_level, self._H)
        return world[0, 0]   # (x, y)

    def _radar_tilt_correction(self, x_m: float, y_m: float,
                               roll_deg: float, pitch_deg: float):
        """
        Radar reports (x, y) assuming horizontal beam.  Roll/pitch tilt it;
        project the tilted unit vector back onto z=0 preserving distance.
        Returns corrected (x, y) in metres.
        """
        R = self._tilt_rotation_matrix(roll_deg, pitch_deg)
        dist = math.hypot(x_m, y_m) or 1e-9
        beam = np.array([x_m / dist, y_m / dist, 0.0])
        beam_world = R @ beam
        bx, by = beam_world[0], beam_world[1]
        scale = dist / (math.hypot(bx, by) or 1e-9)
        return bx * scale, by * scale

    # ------------------------------------------------------------------
    # Public API  (synchronous - reads from .history, never blocks)
    # ------------------------------------------------------------------

    def get_camera_world_points(self):
        """
        Convert latest camera detections to flat-plane world (x, y) metres.
        Uses bottom-centre of each bbox as the ground contact point.

        Returns list of dict with all original fields plus 'world_x', 'world_y'.
        """
        if not self._camera.history['values']:
            return []

        roll, pitch = self._imu_tilt(self._imu.history['values'])

        detections = self._camera.history['values'][-1]
        results = []
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            u = (x1 + x2) / 2.0   # horizontal centre
            v = float(y2)          # bottom edge -> ground contact point

            wx, wy = self._pixel_to_world(u, v, roll, pitch)

            results.append({
                **det,
                'world_x': float(wx),
                'world_y': float(wy),
            })
        return results

    def get_radar_world_points(self):
        """
        Convert latest radar detections (mm, already BEV) to flat-plane world
        (x, y) metres, corrected for roll/pitch tilt.

        Returns list of dict with all original fields plus 'world_x', 'world_y'.
        """
        if not self._radar.history['values']:
            return []

        roll, pitch = self._imu_tilt(self._imu.history['values'])

        targets = self._radar.history['values'][-1]
        results = []
        for t in targets:
            x_m = t['x'] * self._scale
            y_m = t['y'] * self._scale

            wx, wy = self._radar_tilt_correction(x_m, y_m, roll, pitch)

            results.append({
                **t,
                'world_x': float(wx),
                'world_y': float(wy),
            })
        return results

    def get_world_points(self):
        """
        Returns dict with keys 'camera' and 'radar', each a list of dicts
        with 'world_x' and 'world_y' in metres on the flat ground plane.
        """
        return {
            'camera': self.get_camera_world_points(),
            'radar':  self.get_radar_world_points(),
        }

# |-------------------------------------------------------------|
# |----------------- Camera calibration helper -----------------|
# |-------------------------------------------------------------|


@staticmethod
def calibrate_camera(image_paths: list, board_size: tuple = (9, 6),
                     square_size_m: float = 0.025,
                     show_corners: bool = False):
    """
    Compute the camera intrinsic matrix from checkerboard images.

    Parameters
    ----------
    image_paths   : list of file paths to checkerboard images
    board_size    : (cols, rows) inner corners, e.g. (9, 6) for a 10x7 board
    square_size_m : physical size of one square in metres (default 0.025 = 2.5 cm)
    show_corners  : if True, display each image with detected corners

    Returns
    -------
    camera_matrix : (3, 3) ndarray  - intrinsic matrix K
    dist_coeffs   : (1, 5) ndarray  - distortion coefficients
    rms_error     : float           - reprojection RMS error in pixels

    Usage
    -----
    import glob
    K, dist, rms = calibrate_camera(
        image_paths=glob.glob('calib/*.jpg'),
        board_size=(9, 6),
        square_size_m=0.025,
    )
    print('Camera matrix:\\n', K)
    print('RMS reprojection error:', rms)
    # Then pass K as camera_matrix= when constructing BEV.
    """
    cols, rows = board_size

    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_m

    obj_points = []
    img_points = []
    img_size   = None

    for path in image_paths:
        img  = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if not found:
            continue

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        obj_points.append(objp)
        img_points.append(corners)

        if show_corners:
            cv2.drawChessboardCorners(img, (cols, rows), corners, found)
            cv2.imshow('Corners', img)
            cv2.waitKey(300)

    if show_corners:
        cv2.destroyAllWindows()

    if not obj_points:
        raise RuntimeError(
            'No checkerboard corners found in any image. '
            'Check board_size and image paths.'
        )

    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None
    )

    return camera_matrix, dist_coeffs, rms
