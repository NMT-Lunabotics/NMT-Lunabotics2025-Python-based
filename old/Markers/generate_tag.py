import cv2
import numpy as np

# Pick dictionary and ID
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
tag_id = 56
marker_size = 500  # pixels

# Create the marker using the Dictionary object
tag_image = aruco_dict.generateImageMarker(tag_id, marker_size)

# Save to file
cv2.imwrite(f"aruco_tag_{tag_id}.png", tag_image)