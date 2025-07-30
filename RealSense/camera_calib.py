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


def capture_calibration_images(camera_index=0, num_images=20):
    """Capture checkerboard images for calibration"""
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_index}")
        return []
    
    # Set camera resolution for D435i
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    images = []
    count = 0
    
    print(f"Capturing {num_images} calibration images...")
    print("Press SPACE to capture image, ESC to finish")
    
    while count < num_images:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Display frame
        cv2.imshow('Calibration - Press SPACE to capture', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):  # Space key
            filename = f'calib_img_{count:02d}.jpg'
            cv2.imwrite(filename, frame)
            images.append(filename)
            count += 1
            print(f"Captured {filename} ({count}/{num_images})")
        elif key == 27:  # ESC key
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return images

def run_calibration():
    """Run complete camera calibration process"""
    print("D435i Camera Calibration for SCARA ARM")
    print("="*40)
    
    # Option to use existing images or capture new ones
    response = input("Capture new calibration images? (y/n): ").lower()
    
    if response == 'y':
        # Capture new images
        checkerboard_images = capture_calibration_images()
        if not checkerboard_images:
            print("No images captured. Exiting.")
            return
    else:
        # Use existing images
        import glob
        checkerboard_images = glob.glob('calib_img_*.jpg')
        if not checkerboard_images:
            print("No existing calibration images found (calib_img_*.jpg)")
            print("Please capture calibration images first.")
            return
    
    # Checkerboard parameters (adjust based on your checkerboard)
    checkerboard_size = (9, 6)  # inner corners per row, col
    square_size = 0.025         # meters (25mm squares)
    
    print(f"Using {len(checkerboard_images)} images for calibration...")

    try:
        cam_mat, dist_coeffs, rvecs, tvecs = calibrate_camera(
            checkerboard_images, checkerboard_size, square_size
        )

        # Create hand-eye transform for camera mounted on arm
        # Camera facing down, aligned to -x direction of arm
        T_cam_to_robot = np.array([
            [0, -1,  0, 0],    # Camera X maps to Robot -Y
            [0,  0, -1, 0],    # Camera Y maps to Robot -Z (downward)
            [1,  0,  0, 0],    # Camera Z maps to Robot +X (forward)
            [0,  0,  0, 1]
        ])

        # Save parameters to YAML
        params = {
            'camera_matrix': cam_mat.tolist(),
            'dist_coeffs': dist_coeffs.tolist(),
            'T_cam_to_robot': T_cam_to_robot.tolist(),
            'calibration_info': {
                'checkerboard_size': checkerboard_size,
                'square_size': square_size,
                'num_images': len(checkerboard_images),
                'camera_model': 'Intel RealSense D435i'
            }
        }
        
        with open('calib_params.yaml', 'w') as f:
            yaml.dump(params, f)
        
        print("Calibration complete!")
        print(f"Camera matrix:")
        print(cam_mat)
        print(f"Distortion coefficients:")
        print(dist_coeffs.flatten())
        print("Parameters saved to calib_params.yaml")
        
    except Exception as e:
        print(f"Calibration failed: {e}")
        print("Make sure checkerboard is visible in multiple images with different orientations.")

if __name__ == "__main__":
    run_calibration()