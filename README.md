# EXPERIMENT-07 INTERFACING CAMERA MODULE ON EDGE COMPUTER FOR OCCUPANCY DETECTION
### Reg no: 212222110053
### AIM:
To interface a USB/CSI camera module with an edge computing platform (e.g., Raspberry Pi, Jetson Nano, etc.) and implement an occupancy detection system using the Histogram of Oriented Gradients (HOG) algorithm.

### Apparatus/Software Required:
1.	Edge Computing Device	Raspberry Pi 4 / Jetson Nano
2.	Camera Module	USB Webcam / Pi Camera Module
3.	Operating System	Raspbian OS / Ubuntu
4.	Programming Language	Python 3.x
5.	Libraries	OpenCV, imutils, NumPy
6.	Display Output	HDMI Monitor / VNC Viewer

### Theory:
Histogram of Oriented Gradients (HOG) is a feature descriptor used in computer vision and image processing for the purpose of object detection. It counts occurrences of gradient orientation in localized portions of an image. HOG descriptors are particularly useful for detecting humans (pedestrians) in static images or video frames.

## Steps involved in HOG-based Occupancy Detection:

- Capture frames from the camera.
- Resize and preprocess the image.
- Use a pre-trained HOG descriptor with a linear SVM to detect people in the image.
- Annotate the image with bounding boxes where people are detected.
- Display or store the result.

## Circuit Diagram / Setup:
- Connect the USB camera to the edge computer via a USB port.
- Power on the edge device and boot into the OS.
- Ensure necessary Python libraries are installed.

### Procedure:
1. Set up the edge device with a monitor or SSH/VNC connection.
2. Connect and verify the camera using commands like ls /dev/video* or vcgencmd get_camera.
3. Install required libraries:
   - bash
   - Copy
   - Edit
   - pip install opencv-python imutils numpy
4. Write the Python code to initialize the camera and implement the HOG algorithm.
5. Run the code and verify that the system detects human presence and draws bounding boxes.

 ###  Python Code:
 ```
import cv2
import imutils
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
cap = cv2.VideoCapture(0)  # Change index if using CSI camera

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = imutils.resize(frame, width=640)
    (rects, weights) = hog.detectMultiScale(frame, winStride=(4, 4),
                                            padding=(8, 8), scale=1.05)
    for (x, y, w, h) in rects:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.imshow("Occupancy Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```
### Output:
![image](https://github.com/user-attachments/assets/930e110a-8a5e-428c-a838-85b41884f11a)

### RASPI INTERFACE 
![image](https://github.com/user-attachments/assets/f2e9ccbc-f201-409e-953e-571e28fc8e17)

### Result:
Occupancy detection using the HOG algorithm was successfully implemented. The system was able to identify and highlight human presence in real-time video streams.




### Result:
Occupancy detection using the HOG algorithm was successfully implemented. The system was able to identify and highlight human presence in real-time video streams.
