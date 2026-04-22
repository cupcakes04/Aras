

import glob
import numpy as np
import cv2
                   # Small value to prevent division by zero
CALIB_BOARD_SIZE = (4,6)          # Default checkerboard inner corners (cols, rows)
CALIB_SQUARE_SIZE_M = 0.020        # Default physical size of checkerboard square in metres

def calibrate_camera(image_paths: list, board_size: tuple = CALIB_BOARD_SIZE,
                     square_size_m: float = CALIB_SQUARE_SIZE_M,
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
            print('error: image not found')
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if not found:
            continue

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, CALIB_CRITERIA_MAX_ITER, CALIB_CRITERIA_EPS)
        corners = cv2.cornerSubPix(gray, corners, CALIB_SEARCH_WINDOW, (-1, -1), criteria)

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

K, dist, rms = calibrate_camera(
    image_paths=glob.glob('ch/*.jpg'),
    board_size=(9, 6),
    square_size_m=0.025,
)
print('Camera matrix:\n', K)
print('RMS reprojection error:', rms)