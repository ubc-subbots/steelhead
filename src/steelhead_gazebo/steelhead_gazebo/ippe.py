import cv2
import numpy as np

def estimate_camera_pose_ippe(image_points, real_width, real_height, camera_matrix, dist_coeffs=None):
    """
    Estimates the camera position relative to a box face using IPPE.
    
    Parameters:
    - image_points: List/array of 4 2D points [(x,y), ...] ordered as:
                    [Top-Left, Top-Right, Bottom-Left, Bottom-Right]
    - real_width:   The real-world width of the box face (e.g., in centimeters or meters).
    - real_height:  The real-world height of the box face.
    - camera_matrix: The 3x3 camera intrinsic matrix.
    - dist_coeffs:  The camera distortion coefficients (optional).
    
    Returns:
    - R_cam: 3x3 Rotation matrix of the camera relative to the box.
    - t_cam: 3x1 Translation vector (X, Y, Z) of the camera relative to the box center.
    """
    # 1. Define the 3D coordinates of the box face in the object coordinate system.
    # We place the origin (0,0,0) at the center of the box's front face.
    half_w = real_width / 2.0
    half_h = real_height / 2.0
    
    # Object points must be coplanar (Z=0) for IPPE and match the image point order:
    # [Top-Left, Top-Right, Bottom-Left, Bottom-Right]
    object_points = np.array([
        [-half_w,  half_h, 0.0],  # Top-Left
        [ half_w,  half_h, 0.0],  # Top-Right
        [-half_w, -half_h, 0.0],  # Bottom-Left
        [ half_w, -half_h, 0.0]   # Bottom-Right
    ], dtype=np.float32)
    
    # Ensure image points are a float32 numpy array
    image_points = np.array(image_points, dtype=np.float32)
    
    if dist_coeffs is None:
        dist_coeffs = np.zeros((4, 1)) # Assume no distortion if not provided

    # 2. Solve PnP using the IPPE algorithm
    outputs = cv2.solvePnPGeneric(
        objectPoints=object_points,
        imagePoints=image_points,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE
    )
    
    # Check what your OpenCV version returned
    success = outputs[0]
    if not success:
        raise RuntimeError("Pose estimation failed. Check your points and camera matrix.")
        
    # outputs[1] contains the list/array of rotation vectors (rvecs)
    # outputs[2] contains the list/array of translation vectors (tvecs)
    rvecs = outputs[1]
    tvecs = outputs[2]
    
    # Grab the primary (first) solution
    rvec = rvecs[0]
    tvec = tvecs[0]
    
    if not success:
        raise RuntimeError("Pose estimation failed. Check your points and camera matrix.")
        

    # 3. Transform to get the CAMERA's position relative to the BOX
    # solvePnP gives the Box relative to the Camera. We invert this to get Camera relative to Box.
    R_box_to_cam, _ = cv2.Rodrigues(rvec)
    
    # Invert the transformation: R_cam = R_box_to_cam^T, t_cam = -R_box_to_cam^T * tvec
    R_cam = R_box_to_cam.T
    t_cam = -R_cam @ tvec
    
    return R_cam, t_cam

# --- Example Usage ---
if __name__ == "__main__":
    # Dummy Camera Matrix (Replace with your actual calibration data!)
    # Focal lengths (fx, fy) around 800, principal point (cx, cy) around center of a 640x480 image
    K = np.array([[800,   0, 320],
                  [  0, 800, 240],
                  [  0,   0,   1]], dtype=np.float32)
    
    # Detected corners in the image (Top-Left, Top-Right, Bottom-Left, Bottom-Right)
    detected_corners = [
        [200, 150],  # TL
        [420, 140],  # TR
        [180, 360],  # BL
        [440, 350]   # BR
    ]
    
    # Box dimensions in real life (e.g., 20cm x 30cm)
    width = 20.0
    height = 30.0
    
    R, t = estimate_camera_pose_ippe(detected_corners, width, height, K)
    
    print("Camera Rotation Matrix:\n", R)
    print("\nCamera Position (X, Y, Z) relative to box center:\n", t.flatten())