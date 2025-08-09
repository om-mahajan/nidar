# Import Dependencies
import os
import cv2
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from glob import glob
from ultralytics import YOLO
from PIL import Image

""" def geotag():
def px_area(area):
    pixel_size_mm = sensor_width_mm / image_width_px
    gsd_m = (altitude_agl_m * pixel_size_mm) / focal_length_mm / 1000.0  # mm->m
    area_m2 = (bbox_w_px * gsd_m) * (bbox_h_px * gsd_m) """

trained_model = YOLO("best_tomato_leaf_model.pt")

# Define test images path
TEST_IMAGE_DIR = "D:/nidar_model/yolo8_without_blur/test/images"
AGL = 10 #PIXHAWK ALT - MSL
# Run inference on test images
test_images = glob(os.path.join(TEST_IMAGE_DIR, "*.jpg"))[2:20]

# Display results

for i, img_path in enumerate(test_images):
    img = Image.open(img_path)
    
    # Perform inference
    results = trained_model(img)
    
    # Draw bounding boxes
    results[0].show()
    img_np = np.array(img)
    
    
    boxes = results[0].boxes 
    
    for box in boxes:
        
        x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
        x_c = (x_min + x_max)/2
        y_c = (y_min + y_max)/2
        area = (x_max-x_min)*(y_max-y_min)
    
        conf = box.conf[0].item()
        
        
        cls_id = int(box.cls[0].item())
        cls_name = results[0].names[cls_id]
        
        print(f"Image {i+1}: {cls_name} ({conf:.2f})")
        print(f"Pixel coordinates: ({x_c:.0f}, {y_c:.0f})\n")

