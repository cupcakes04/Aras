# ARAS 3D Visualization

Real-time 3D visualization of multi-object tracking using Three.js.

## Features

- **3D Bird's Eye View**: Objects displayed in 3D space with realistic dimensions
- **Real-time Tracking**: Live updates via WebSocket at 20Hz
- **Object Classes**: Different colors and sizes for each object type:
  - 🚗 Car (green)
  - 🚐 Van (blue)
  - 🚚 Truck (orange)
  - 🚶 Pedestrian (yellow)
  - 🚴 Cyclist (purple)
  - 🚊 Tram (cyan)
  - 📦 Misc (gray)
- **Track States**: Visual distinction between confirmed (solid) and tentative (transparent) tracks
- **Velocity Vectors**: Real-time speed and direction display
- **Motion Trails**: Optional path history visualization
- **Interactive Controls**: Orbit camera, toggle grid, labels, and trails

## Object Dimensions

Each object class has realistic 3D dimensions (width × height × depth):

| Class | Width | Height | Depth | Color |
|-------|-------|--------|-------|-------|
| Car | 1.8m | 1.5m | 4.5m | Green |
| Van | 2.0m | 2.2m | 5.0m | Blue |
| Truck | 2.5m | 3.0m | 8.0m | Orange |
| Pedestrian | 0.5m | 1.7m | 0.3m | Yellow |
| Person Sitting | 0.5m | 1.0m | 0.5m | Amber |
| Cyclist | 0.6m | 1.7m | 1.8m | Purple |
| Tram | 2.5m | 3.5m | 15.0m | Cyan |
| Misc | 1.0m | 1.0m | 1.0m | Gray |

## Coordinate System

- **BEV Coordinates**: (x, y) where +y is forward, +x is right
- **Three.js Mapping**: 
  - x → x (right)
  - y → height
  - z → -y (forward, negated for Three.js convention)
- **Origin**: Ego vehicle (bike) at (0, 0, 0)

## Usage

### 1. Install Dependencies

```bash
pip install websockets
```

### 2. Run the System

From the `system` directory:

```bash
python run.py
```

This will:
- Start the ARAS tracking system
- Launch WebSocket server on `ws://localhost:8765`
- Begin streaming tracking data at 20Hz
- Display console status reports

### 3. Open Visualization

Simply open `app/index.html` in a web browser (double-click or drag to browser).

The visualization will automatically connect to `ws://localhost:8765`.

## Controls

### Camera
- **Left Mouse**: Rotate view
- **Right Mouse**: Pan
- **Scroll**: Zoom in/out
- **Reset Camera**: Return to default view

### Display Options
- **Toggle Grid**: Show/hide ground grid
- **Toggle Labels**: Show/hide object name labels
- **Toggle Trails**: Show/hide motion history trails

## Info Panel

The left panel displays:
- **Connection Status**: WebSocket connection state
- **Tracked Objects**: Count of confirmed and tentative tracks
- **Update Rate**: Data stream frequency (Hz)
- **Object List**: Detailed info for each tracked object:
  - ID and class name
  - Position (x, y) in meters
  - Velocity magnitude
  - Confidence score
  - Hit count

## Architecture

```
┌─────────────────┐
│  ARAS System    │
│  (tracker.py)   │
└────────┬────────┘
         │
         │ Tracking Data
         ▼
┌─────────────────┐
│ WebSocket Server│
│  (server.py)    │
└────────┬────────┘
         │
         │ JSON Stream (20Hz)
         ▼
┌─────────────────┐
│  Three.js Web   │
│  (index.html)   │
└─────────────────┘
```

## Data Format

WebSocket messages contain:

```json
{
  "tracks": [
    {
      "id": 1,
      "name": "car",
      "world_x": 2.5,
      "world_y": 5.0,
      "vx": 0.1,
      "vy": 2.0,
      "confidence": 0.95,
      "cam_only": false,
      "hits": 15,
      "misses": 0,
      "age": 15,
      "state": "confirmed"
    }
  ],
  "timestamp": 1234567890.123
}
```

## Troubleshooting

### Connection Issues
- Ensure `server.py` is running
- Check WebSocket URL is `ws://localhost:8765`
- Verify no firewall blocking port 8765

### No Objects Visible
- Check that tracking system is receiving sensor data
- Verify objects are in confirmed state (3+ hits)
- Check console for JavaScript errors

### Performance Issues
- Reduce trail length (currently 50 points)
- Disable trails if many objects
- Lower WebSocket update rate in `server.py`

## Future Enhancements

- [ ] Collision zone visualization (front/side threat areas)
- [ ] Velocity vector arrows
- [ ] Object bounding box uncertainty ellipses
- [ ] Playback controls (pause, rewind)
- [ ] Data recording and replay
- [ ] Multiple camera views (top, side, perspective)
- [ ] Heatmap of object density
