import asyncio
import random
import time
import cv2
import os
import json
import subprocess
from collections import deque
from .history import History

class CameraOld(History):
    """
    Camera
    - functions in reading frames 
    - data of 

    
    Example output from a library function like radar.get_targets(), each id can have multiple dicts
    ```python
        [
            {'class': 2, 'bbox': [x1,y1,x2,y2], 'name': 'car'},
            {'class': 4, 'bbox': [x1,y1,x2,y2], 'name': 'truck'},
        ]
    ```
    """

    
    def __init__(self, config={}, **kwargs):
        super().__init__(**kwargs)
        self.setup(**kwargs)
        
    def setup(self, **kwargs):
        self.frames = []
        self.frame_idx = 0
        try:
            # Try loading the demo json file
            demo_path = os.path.join(os.path.dirname(__file__), '../../assets/demo/result.json')
            with open(demo_path, 'r') as f:
                self.frames = json.load(f)
            print(f"[CameraOld] Loaded {len(self.frames)} frames from demo JSON.")
        except Exception as e:
            print(f"[CameraOld] Could not load demo JSON: {e}. Falling back to hardcoded dummy data.")
            self.frames = [{
                'objs': [
                # Person slightly to the left, medium distance (bottom edge ~450)
                    {'class': 0, 'bbox': [150, 200, 250, 450], 'name': 'person'},
                # Car straight ahead, further away (bottom edge ~380)
                    {'class': 2, 'bbox': [280, 250, 360, 380], 'name': 'car'},
                # Truck to the right, closer (bottom edge ~550)
                    {'class': 5, 'bbox': [400, 150, 600, 550], 'name': 'truck'},
                ],
                'signs': [
                # Traffic light high up, further away (bottom edge ~250)
                    {'class': 0, 'bbox': [300, 50, 340, 250], 'name': 'traffic light'},
                # Stop sign on the right side, medium distance (bottom edge ~420)
                    {'class': 1, 'bbox': [500, 200, 550, 420], 'name': 'stop sign'},
                ],
            }]
        
    async def read(self):
        if not self.frames:
            return
            
        value = self.frames[self.frame_idx]
        self.frame_idx = (self.frame_idx + 1) % len(self.frames)

        self.save_history(value)


class Camera(History):
    """
    Hardware Camera reading from GStreamer and running YOLOv5 via NPU subprocess.
    """
    def __init__(self, config={}, cam_device="/dev/video1", **kwargs):
        super().__init__(**kwargs)

        self.objs_class_map = config.get('objs_class_map', {})
        self.signs_class_map = config.get('signs_class_map', {})
            
        self.setup(cam_device)
        
    def setup(self, cam_device):
        self._hardware_available = True
        
        # GStreamer pipeline for Radxa
        gst_str = (
            f"v4l2src device={cam_device} en-awisp=1 en-largemode=1 ! "
            "video/x-raw,format=NV12,width=640,height=640,framerate=24/1 ! "
            "videoconvert ! "
            "videoscale ! "
            "video/x-raw,width=640,height=640 ! "
            "appsink drop=true"
        )
        
        try:
            self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
            if not self.cap.isOpened():
                raise Exception("Could not open GStreamer pipeline")
        except Exception as e:
            print(f"[Camera] Hardware init failed: {e}. Running in dummy mode.")
            self._hardware_available = False
            
        # Paths for NPU execution
        self.SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
        self.YOLO_SCRIPT = os.path.join(self.SCRIPT_DIR, "./ai-sdk/examples/yolov5")
        self.YOLO_EXE = "./yolov5"
        self.MODEL_PATH = "./model/v3/yolov5.nb"
        
        # Create input directory if it doesn't exist
        self.IMAGE_PATH = os.path.join(SCRIPT_DIR, "./input_data/cur_frame.jpg") 
        self.RES_PATH = os.path.join(YOLO_SCRIPT, "./results.json") 

    def close(self):
        if self._hardware_available and hasattr(self, 'cap'):
            self.cap.release()

    async def read(self):
        if not self._hardware_available:
            # Fallback to dummy data
            value = {'objs': [], 'signs': []}
            self.save_history(value)
            return

        ret, frame = self.cap.read()
        if not ret:
            self.save_history({'objs': [], 'signs': []})
            return

        # Save frame for YOLO
        cv2.imwrite(self.IMAGE_PATH, frame)

        # Run YOLO synchronously (this might block asyncio, consider run_in_executor in production)
        try:
            subprocess.run(
                [self.YOLO_EXE, self.MODEL_PATH, self.IMAGE_PATH],
                cwd=self.YOLO_SCRIPT,
                capture_output=True,
                text=True
            )
            
            with open(self.RES_PATH) as f:
                raw_results = json.load(f)
                
            # Process results into standardized format
            objs = []
            signs = []
            
            for res in raw_results:
                label_name = res['label']
                # Map label string to ID
                class_id = 7 # default misc
                is_sign = False
                
                for k, v in self.objs_class_map.items():
                    if v == label_name:
                        class_id = k
                        break
                for k, v in self.signs_class_map.items():
                    if v == label_name:
                        class_id = k
                        is_sign = True
                        break
                        
                box = res['box']
                x, y, w, h = box['x'], box['y'], box['w'], box['h']
                
                item = {
                    'class': class_id,
                    'name': label_name,
                    'bbox': [x, y, x+w, y+h],
                    'confidence': res['confidence']
                }
                
                if is_sign:
                    signs.append(item)
                else:
                    objs.append(item)
                
            value = {'objs': objs, 'signs': signs}
            
        except Exception as e:
            print(f"[Camera] Inference error: {e}")
            value = {'objs': [], 'signs': []}

        self.save_history(value)