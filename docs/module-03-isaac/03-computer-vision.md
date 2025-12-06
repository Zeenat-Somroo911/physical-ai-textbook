---
sidebar_position: 3
---

# Chapter 3: Computer Vision with OpenCV

## Introduction

OpenCV (Open Source Computer Vision Library) is the industry-standard library for computer vision. It's completely free, open-source, and works on any computer without special hardware.

### Why OpenCV?

- **Free**: Completely open-source
- **Cross-platform**: Windows, Linux, macOS
- **Extensive**: 2500+ algorithms
- **Well-documented**: Excellent resources
- **Fast**: Optimized C++ with Python bindings
- **No GPU Required**: Works on CPU

## Installation

### Basic Installation

```bash
# Install OpenCV
pip install opencv-python

# Install with contrib modules (extra features)
pip install opencv-contrib-python

# Verify installation
python3 -c "import cv2; print(cv2.__version__)"
```

### Installation with ROS 2

```bash
# OpenCV is usually pre-installed with ROS 2
# If not:
sudo apt install python3-opencv

# For C++:
sudo apt install libopencv-dev
```

## Basic OpenCV Usage

### Reading and Displaying Images

```python
import cv2
import numpy as np

# Read image
img = cv2.imread('image.jpg')

# Display image
cv2.imshow('Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Save image
cv2.imwrite('output.jpg', img)
```

### Working with Camera

```python
import cv2

# Open camera
cap = cv2.VideoCapture(0)  # 0 for default camera

while True:
    # Read frame
    ret, frame = cap.read()
    
    if not ret:
        break
    
    # Process frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Display
    cv2.imshow('Camera', frame)
    
    # Exit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release camera
cap.release()
cv2.destroyAllWindows()
```

## Camera Calibration

Camera calibration is essential for accurate measurements and 3D reconstruction.

### Calibration Process

```python
#!/usr/bin/env python3
"""
Camera Calibration

Calibrate camera using checkerboard pattern.
"""

import cv2
import numpy as np
import glob

# Checkerboard dimensions (inner corners)
CHECKERBOARD = (9, 6)  # 9x6 inner corners

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Arrays to store object and image points
objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image plane

# Load calibration images
images = glob.glob('calibration_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find checkerboard corners
    ret, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH +
        cv2.CALIB_CB_FAST_CHECK +
        cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    
    if ret:
        objpoints.append(objp)
        
        # Refine corners
        corners2 = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        imgpoints.append(corners2)
        
        # Draw corners
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('Calibration', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    gray.shape[::-1],
    None,
    None
)

# Save calibration
np.savez('camera_calibration.npz', mtx=mtx, dist=dist)

print("Camera matrix:")
print(mtx)
print("\nDistortion coefficients:")
print(dist)
```

### Using Calibration

```python
# Load calibration
calibration = np.load('camera_calibration.npz')
mtx = calibration['mtx']
dist = calibration['dist']

# Undistort image
img = cv2.imread('test_image.jpg')
undistorted = cv2.undistort(img, mtx, dist, None, mtx)
```

## Object Detection with YOLO

YOLO (You Only Look Once) is a state-of-the-art object detection system. We'll use free, pre-trained models.

### Installation

```bash
# Install ultralytics (YOLOv8)
pip install ultralytics

# Or use OpenCV DNN with YOLO
# Download weights from: https://github.com/AlexeyAB/darknet
```

### YOLO Detection with Ultralytics

```python
#!/usr/bin/env python3
"""
YOLO Object Detection

Detect objects using free YOLO models.
"""

from ultralytics import YOLO
import cv2

# Load pre-trained model (free, downloads automatically)
model = YOLO('yolov8n.pt')  # nano version (fastest, smallest)

# Or use other versions:
# model = YOLO('yolov8s.pt')  # small
# model = YOLO('yolov8m.pt')  # medium
# model = YOLO('yolov8l.pt')  # large
# model = YOLO('yolov8x.pt')  # extra large

# Open camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Run detection
    results = model(frame)
    
    # Draw results
    annotated_frame = results[0].plot()
    
    # Display
    cv2.imshow('YOLO Detection', annotated_frame)
    
    # Print detections
    for result in results:
        boxes = result.boxes
        for box in boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            label = model.names[cls]
            print(f"Detected: {label} with confidence: {conf:.2f}")
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### YOLO with OpenCV DNN

```python
import cv2
import numpy as np

# Load YOLO
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Load class names
with open("coco.names", "r") as f:
    classes = [line.strip() for f.readlines()]

# Load image
img = cv2.imread("image.jpg")
height, width, channels = img.shape

# Prepare blob
blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
net.setInput(blob)
outs = net.forward(output_layers)

# Process detections
for out in outs:
    for detection in out:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        
        if confidence > 0.5:
            # Get bounding box
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[2] * width)
            h = int(detection[3] * height)
            
            # Draw box
            cv2.rectangle(img, (center_x - w//2, center_y - h//2),
                         (center_x + w//2, center_y + h//2), (0, 255, 0), 2)
            
            # Draw label
            label = f"{classes[class_id]}: {confidence:.2f}"
            cv2.putText(img, label, (center_x, center_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

cv2.imshow("Detection", img)
cv2.waitKey(0)
```

## Pose Estimation

### MediaPipe Pose Estimation (Free)

```python
#!/usr/bin/env python3
"""
Pose Estimation with MediaPipe

Free pose estimation for humans and objects.
"""

import cv2
import mediapipe as mp

# Install: pip install mediapipe

mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils

# Initialize pose estimation
pose = mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert BGR to RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Process frame
    results = pose.process(rgb_frame)
    
    # Draw pose
    if results.pose_landmarks:
        mp_drawing.draw_landmarks(
            frame,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS
        )
    
    cv2.imshow('Pose Estimation', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
pose.close()
cv2.destroyAllWindows()
```

## Face Detection

### Haar Cascades (Built-in, Free)

```python
#!/usr/bin/env python3
"""
Face Detection

Detect faces using OpenCV's built-in Haar cascades.
"""

import cv2

# Load face cascade (included with OpenCV)
face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect faces
    faces = face_cascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30)
    )
    
    # Draw rectangles around faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.putText(frame, 'Face', (x, y-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
    
    cv2.imshow('Face Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

## Object Tracking

### MeanShift Tracking

```python
#!/usr/bin/env python3
"""
Object Tracking with MeanShift

Track objects in video stream.
"""

import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# Read first frame
ret, frame = cap.read()

# Select ROI (Region of Interest)
r = cv2.selectROI("Select Object", frame, False)
track_window = r

# Setup tracking
hsv_roi = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv_roi, np.array((0., 60., 32.)),
                   np.array((180., 255., 255.)))
roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)

term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    dst = cv2.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)
    
    # Apply MeanShift
    ret, track_window = cv2.meanShift(dst, track_window, term_crit)
    
    # Draw tracking box
    x, y, w, h = track_window
    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    
    cv2.imshow('Tracking', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### CSRT Tracker

```python
import cv2

# Create tracker
tracker = cv2.TrackerCSRT_create()

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

# Select ROI
bbox = cv2.selectROI("Select Object", frame, False)
tracker.init(frame, bbox)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Update tracker
    success, bbox = tracker.update(frame)
    
    if success:
        x, y, w, h = [int(v) for v in bbox]
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    
    cv2.imshow('CSRT Tracking', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

## ArUco Markers

ArUco markers are fiducial markers used for pose estimation and camera calibration.

### Detection

```python
#!/usr/bin/env python3
"""
ArUco Marker Detection

Detect and estimate pose of ArUco markers.
"""

import cv2
import numpy as np

# Define ArUco dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters_create()

# Camera calibration (from previous section)
mtx = np.load('camera_calibration.npz')['mtx']
dist = np.load('camera_calibration.npz')['dist']

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Detect markers
    corners, ids, rejected = cv2.aruco.detectMarkers(
        frame, aruco_dict, parameters=aruco_params
    )
    
    if ids is not None:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Estimate pose (if marker size known)
        marker_size = 0.05  # 5cm
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, mtx, dist
        )
        
        # Draw axis
        for i in range(len(ids)):
            cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.03)
            
            # Print pose
            print(f"Marker {ids[i][0]}:")
            print(f"  Translation: {tvecs[i][0]}")
            print(f"  Rotation: {rvecs[i][0]}")
    
    cv2.imshow('ArUco Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### Generating Markers

```python
import cv2

# Generate ArUco marker
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
marker_id = 0
marker_size = 200

marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)
cv2.imwrite(f'marker_{marker_id}.png', marker_img)
cv2.imshow('Marker', marker_img)
cv2.waitKey(0)
```

## Depth Estimation

### Stereo Vision

```python
#!/usr/bin/env python3
"""
Stereo Depth Estimation

Estimate depth using stereo cameras.
"""

import cv2
import numpy as np

# Stereo calibration (requires calibration of both cameras)
# Load calibration
stereo_calib = np.load('stereo_calibration.npz')

# Create stereo matcher
stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)

# Open cameras (assuming two cameras)
cap_left = cv2.VideoCapture(0)
cap_right = cv2.VideoCapture(1)

while True:
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()
    
    if not (ret_left and ret_right):
        break
    
    # Convert to grayscale
    gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)
    
    # Compute disparity
    disparity = stereo.compute(gray_left, gray_right)
    
    # Normalize for display
    disparity_normalized = cv2.normalize(
        disparity, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
    )
    
    # Apply colormap
    disparity_colored = cv2.applyColorMap(disparity_normalized, cv2.COLORMAP_JET)
    
    cv2.imshow('Left', frame_left)
    cv2.imshow('Right', frame_right)
    cv2.imshow('Depth', disparity_colored)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
```

## Complete Perception Pipeline

### ROS 2 Vision Node

```python
#!/usr/bin/env python3
"""
Complete Vision Pipeline

Combines detection, tracking, and pose estimation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class VisionPipeline(Node):
    def __init__(self):
        super().__init__('vision_pipeline')
        
        # Initialize components
        self.bridge = CvBridge()
        self.yolo_model = YOLO('yolov8n.pt')
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(Image, '/vision/detections', 10)
        
        self.get_logger().info('Vision pipeline started')
    
    def image_callback(self, msg):
        """Process incoming image."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLO detection
            results = self.yolo_model(cv_image)
            
            # Draw results
            annotated = results[0].plot()
            
            # Convert back to ROS
            ros_image = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            ros_image.header = msg.header
            
            # Publish
            self.detection_pub.publish(ros_image)
            
            # Log detections
            for result in results:
                for box in result.boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = self.yolo_model.names[cls]
                    self.get_logger().info(
                        f"Detected: {label} ({conf:.2f})"
                    )
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionPipeline()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Best Practices

1. **Use appropriate models**: Choose YOLO size based on speed/accuracy needs
2. **Optimize for real-time**: Reduce image resolution if needed
3. **Cache models**: Load models once, reuse
4. **Error handling**: Always handle camera/image errors
5. **Resource management**: Release cameras and windows properly

## Common Errors and Solutions

### Error 1: "cv2 module not found"

```bash
# Solution
pip install opencv-python
```

### Error 2: "Camera not opening"

```python
# Solution: Check camera index
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera found at index {i}")
        break
```

### Error 3: "YOLO model not found"

```python
# Solution: Model downloads automatically on first use
# Or download manually from ultralytics website
```

## Next Steps

Continue learning:
- [Chapter 4: SLAM Basics](04-slam-basics.md) - Build maps for free
- [Chapter 5: Navigation](05-navigation.md) - Navigate autonomously

