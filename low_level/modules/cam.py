import cv2
import os
import numpy as np
import json
import subprocess
import time
import os
# from PIL import Image


def draw_json_annotations(image, results):
    """
    image: numpy array (H, W, 3)
    results: list of dicts containing 'label', 'confidence', and 'box'


    Usage example:
    # img = cv2.imread("your_frame.jpg")
    # results = [{'label': 'tennis racket', 'confidence': 0.505475, 'box': {'x': 1.78879, 'y': 1.25737, 'w': 71.9028, 'h': 155.182}}]
    # annotated_img = draw_json_annotations(img, results)
    # cv2.imshow("Result", annotated_img)
    # cv2.waitKey(0)

    """
    for res in results:
        label = res['label']
        conf = res['confidence']
        box = res['box']

        # 1. Convert coordinates to integers
        x = int(box['x'])
        y = int(box['y'])
        w = int(box['w'])
        h = int(box['h'])

        # 2. Define the color (BGR) and text
        color = (0, 255, 0) # Green
        text = f"{label} {conf:.2f}"

        # 3. Draw the Bounding Box
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)

        # 4. Draw the Label Background (filled rectangle)
        (text_w, text_h), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(image, (x, y - text_h - baseline), (x + text_w, y), color, -1)

        # 5. Put the Text
        cv2.putText(image, text, (x, y - baseline), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    
    return image


# Optimized for 640x640 directly from hardware
# We add 'videoscale' and 'video/x-raw,width=640,height=640' to the pipeline
gst_str_1 = (
    "v4l2src device=/dev/video1 en-awisp=1 en-largemode=1 ! "
    "video/x-raw,format=NV12,width=640,height=640,framerate=24/1 ! "
    "videoconvert ! "
    "videoscale ! "
    "video/x-raw,width=640,height=640 ! "
    "appsink drop=true"
)

gst_str_2 = (
    "v4l2src device=/dev/video1 en-awisp=1 en-largemode=1 ! "
    "video/x-raw,format=NV12,width=640,height=640,framerate=24/1 ! "
    "videoconvert ! "
    "videoscale ! "
    "video/x-raw,width=640,height=640,format=BGR ! "
    "appsink drop=true sync=false"
)

gst_str = gst_str_1

save_path = "./input_data/cur_frame.jpg"

cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)


# $ /usr/bin/python3 cam.py

# 1. Setup Absolute Paths (This prevents "File Not Found" errors)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
YOLO_SCRIPT = os.path.join(SCRIPT_DIR, "./ai-sdk/examples/yolov5")
YOLO_EXE = "./yolov5"  # Kept as relative because we use cwd
MODEL_PATH = "./model/v3/yolov5.nb"
IMAGE_PATH = os.path.join(SCRIPT_DIR, "./input_data/cur_frame.jpg") 
RES_PATH = os.path.join(YOLO_SCRIPT, "./results.json") 

if not cap.isOpened():
    print("Error: Could not open GStreamer pipeline.")
    exit()

try:
    print("Starting capture... Press Ctrl+C to stop.")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        print(frame.shape, frame.max(), frame.min())
        # Save the frame for YOLOv5 to pick up
        # frame = frame[:, :, [0, 2, 1]]
        cv2.imwrite(save_path, frame)

        # # Run YOLO with the correct Working Directory
        result = subprocess.run(
            [YOLO_EXE, MODEL_PATH, IMAGE_PATH],
            cwd=YOLO_SCRIPT,    # This is the "cd" equivalent
            capture_output=True,
            text=True
        )
        with open(RES_PATH) as f:
            res = json.load(f)
            print(res)

        print(result)


        # pil_img = Image.fromarray(frame)
        # pil_img.save("test_pil.jpg")
        
        # Optional: Show locally to verify
        
        cv2.imshow("Annotated", draw_json_annotations(frame, res))
        # cv2.imshow("Annotated", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()