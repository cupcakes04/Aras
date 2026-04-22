import cv2
import numpy as np
import sys
import os

# ── Config ──────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_PATH = os.path.join(SCRIPT_DIR, "./mainbox.jpg")
# ────────────────────────────────────────────────────────

points = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        print(f"  [{len(points)-1}] x={x}, y={y}")
        redraw()

    elif event == cv2.EVENT_RBUTTONDOWN:
        if points:
            removed = points.pop()
            print(f"  Removed: {removed}")
            redraw()

def redraw():
    canvas = base_img.copy()
    for i, (x, y) in enumerate(points):
        cv2.circle(canvas, (x, y), 5, (0, 255, 0), -1)
        cv2.putText(canvas, f"{i}({x},{y})", (x + 8, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    # Draw polygon if 4+ points
    if len(points) >= 4:
        pts = np.array(points, dtype=np.int32)
        cv2.polylines(canvas, [pts], isClosed=True, color=(255, 100, 0), thickness=1)
    cv2.imshow("Point Picker", canvas)

img = cv2.imread(IMAGE_PATH)
if img is None:
    print(f"Cannot open image: {IMAGE_PATH}")
    sys.exit(1)

base_img = img.copy()

cv2.namedWindow("Point Picker")
cv2.setMouseCallback("Point Picker", mouse_callback)
redraw()

print("Left click  : add point")
print("Right click : remove last point")
print("'s'         : save to npy")
print("'c'         : clear all")
print("'q'         : quit")

while True:
    key = cv2.waitKey(0) & 0xFF

    if key == ord('s'):
        arr = np.array(points, dtype=np.float32)
        print(f"Saved {len(points)} points → {arr}")
        print(arr)

    elif key == ord('c'):
        points.clear()
        print("Cleared all points")
        redraw()

    elif key == ord('q'):
        break

cv2.destroyAllWindows()