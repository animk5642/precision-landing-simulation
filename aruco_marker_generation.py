import cv2
import numpy as np

# 1. Load dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# 2. Generate marker (600x600)
marker_image = cv2.aruco.generateImageMarker(aruco_dict, 0, 600)

# 3. Add a 100-pixel white border (Quiet Zone) around it
marker_with_border = cv2.copyMakeBorder(
    marker_image, 
    100, 100, 100, 100, 
    cv2.BORDER_CONSTANT, 
    value=[255, 255, 255]
)

# 4. Save it
save_path = "/home/anirudh/ardupilot_gazebo/models/marker_0.png"
cv2.imwrite(save_path, marker_with_border)
print(f"Marker saved with border at {save_path}")