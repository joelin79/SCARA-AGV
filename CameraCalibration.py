import cv2
import numpy as np
import yaml

# === Calibration Pipeline (Run Once) ===
def calibrate_camera(checkerboard_images, checkerboard_size, square_size):
    """
    Calibrate camera intrinsics and extrinsics using checkerboard images.
    Returns: camera_matrix, dist_coeffs, rvecs, tvecs
    """
    objp = np.zeros((checkerboard_size[0]*checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.indices(checkerboard_size).T.reshape(-1, 2) * square_size

    objpoints, imgpoints = [], []
    for img_path in checkerboard_images:
        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

    _, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    return camera_matrix, dist_coeffs, rvecs, tvecs


def run_calibration():
    # User-provided checkerboard image paths and parameters
    checkerboard_images = ['img1.jpg', 'img2.jpg', 'img3.jpg']
    checkerboard_size = (9, 6)  # inner corners per row, col
    square_size = 0.025        # meters (size of one square)

    cam_mat, dist_coeffs, rvecs, tvecs = calibrate_camera(
        checkerboard_images, checkerboard_size, square_size
    )

    # Placeholder: compute or provide hand-eye transform T_cam_to_robot
    T_cam_to_robot = np.eye(4)  # replace with your solved transform

    # Save parameters to YAML
    params = {
        'camera_matrix': cam_mat.tolist(),
        'dist_coeffs': dist_coeffs.tolist(),
        'T_cam_to_robot': T_cam_to_robot.tolist()
    }
    with open('calib_params.yaml', 'w') as f:
        yaml.dump(params, f)
    print("Calibration complete. Parameters saved to calib_params.yaml")