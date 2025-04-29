import cv2
import numpy as np

# Define ArUco dictionary and marker ID
aruco_dict_name = cv2.aruco.DICT_ARUCO_ORIGINAL  # Change this if needed
marker_id = 100  # Change this to the desired marker ID
marker_size = 150  # Size in pixels

# Get the dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)

# Create an empty image
marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)

# Generate the marker
cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, marker_image, 1)

# Save the marker as an image file
marker_filename = f"Marker{marker_id}.png"
cv2.imwrite(marker_filename, marker_image)

print(f"ArUco marker ID {marker_id} saved as {marker_filename}")

# for i in range(0,10):
#     # Get the dictionary
#     aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)

#     # Create an empty image
#     marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)

#     # Generate the marker
#     cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, marker_image, 1)

#     # Save the marker as an image file
#     marker_filename = f"Marker{marker_id}.png"
#     cv2.imwrite(marker_filename, marker_image)

#     print(f"ArUco marker ID {marker_id} saved as {marker_filename}")
#     marker_id += 1

# Display the marker
# cv2.imshow("ArUco Marker", marker_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()