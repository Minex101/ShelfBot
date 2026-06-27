import cv2

# Define the ArUco dictionary to use
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Generate a 4x4 ArUco marker with ID 0
marker_id = 0
marker_size = 200  # Size of the marker in pixels
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

cv2.imwrite("aruco_marker_0.png", marker_image)
print("ArUco marker with ID 0 generated and saved as 'aruco_marker_0.png'.")