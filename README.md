# Drone Path Tracking Documentation

## Project Overview

This project implements a computer vision-based drone trajectory reconstruction system that extracts flight paths from video footage and converts them into GPS coordinates. The system uses optical flow analysis to track camera movement and compensates for drone rotation to accurately map the flight path.

---

## Approach

### 1. **Optical Flow Tracking**

The system uses the **Lucas-Kanade optical flow algorithm** to track feature points between consecutive video frames:

- **Feature Detection**: `cv2.goodFeaturesToTrack()` identifies distinctive points (corners, edges) in the first frame
- **Point Tracking**: `cv2.calcOpticalFlowPyrLK()` tracks these points across frames using pyramidal implementation
- **Motion Estimation**: Median displacement of tracked points represents camera movement

### 2. **Rotation Compensation**

Unlike simple translation-only tracking, this implementation accounts for drone rotation:

- **Affine Transformation**: `cv2.estimateAffinePartial2D()` estimates the full transformation matrix between frames
- **Rotation Extraction**: Rotation angle is extracted from the transformation matrix: `θ = atan2(M[1,0], M[0,0])`
- **Accumulated Heading**: Total rotation is accumulated across all frames to maintain global orientation

### 3. **Coordinate System Transformation**

The pipeline involves three coordinate transformations:

```
Camera Pixels → Local Meters → Global Meters → GPS Coordinates
```

**Step 1: Pixels to Local Meters**
```
dx_meters = dx_pixels × GSD
dy_meters = dy_pixels × GSD
```
Where GSD (Ground Sample Distance) = 0.023 m/pixel for 25m altitude

**Step 2: Local to Global (Rotation Compensation)**
```
dx_global = dx_local × cos(θ) - dy_local × sin(θ)
dy_global = dx_local × sin(θ) + dy_local × cos(θ)
```

**Step 3: Meters to GPS**
```
Δlat = movement_north / 111,111 meters
Δlon = movement_east / (111,111 × cos(latitude))
```

### 4. **Trajectory Smoothing**

Raw trajectories contain noise from:
- Optical flow estimation errors
- Feature detection instabilities
- Camera vibrations

Solution: **Gaussian filtering** with σ=2 applied independently to latitude and longitude series.

---

## Tools and Libraries Used

### Core Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| **OpenCV** | cv2 | Video processing, optical flow, feature detection |
| **NumPy** | numpy | Numerical computations, array operations |
| **SciPy** | scipy.ndimage | Gaussian filtering for trajectory smoothing |
| **Folium** | folium | Interactive map visualization |
| **Matplotlib** | pyplot | Additional plotting (if needed) |

### Key OpenCV Functions

- `cv2.VideoCapture()` - Video file reading
- `cv2.goodFeaturesToTrack()` - Shi-Tomasi corner detection
- `cv2.calcOpticalFlowPyrLK()` - Lucas-Kanade optical flow
- `cv2.estimateAffinePartial2D()` - Robust transformation estimation with RANSAC
- `cv2.cvtColor()` - Color space conversion (BGR to grayscale)

---

## Algorithm Parameters

### Feature Detection
```python
maxCorners = 300        # Maximum features to track
qualityLevel = 0.3      # Minimum quality of corners (0-1)
minDistance = 7         # Minimum pixel distance between features
blockSize = 7           # Size of averaging block
```

### Optical Flow
```python
winSize = (21, 21)      # Search window size
maxLevel = 2            # Pyramid levels for multi-scale search
criteria = (EPS|COUNT, 30, 0.01)  # Termination criteria
```

### Transformation Estimation
```python
method = cv2.RANSAC     # Robust estimation method
ransacReprojThreshold = 3.0  # Maximum pixel error for inliers
```

### Calibration
```python
GSD = 0.023 m/pixel     # Calculated for 25m altitude
smoothing_sigma = 2.0   # Gaussian filter parameter
```

---

## Processing Pipeline

```
[Video Input]
     ↓
[Extract Frame] → [Convert to Grayscale]
     ↓
[Detect Features (Shi-Tomasi Corners)]
     ↓
[Calculate Optical Flow (Lucas-Kanade)]
     ↓
[Estimate Affine Transform (RANSAC)]
     ↓
[Extract Translation (dx, dy) & Rotation (dθ)]
     ↓
[Compensate for Rotation: Rotate vector by accumulated θ]
     ↓
[Convert Pixels → Meters (multiply by GSD)]
     ↓
[Convert Meters → GPS (divide by Earth metrics)]
     ↓
[Accumulate to Trajectory List]
     ↓
[More Frames?] → Yes → Loop back to Extract Frame
     ↓ No
[Smooth Trajectory (Gaussian Filter)]
     ↓
[Generate Interactive Map (Folium)]
     ↓
[Output: drone_path.html]
```

---

## Results

### Test Flight Data

**Starting Position:**
- GPS: (48.2658°, 25.9184°)
- Altitude: 25 meters

**Ending Position (Ground Truth) (from video):**
- GPS: (48.2659°, 25.9192°)

**Calculated Ending Position:**
- GPS: (varies based on calibration)

### Performance Metrics

| Metric | Value |
|--------|-------|
| Total Frames Processed | ~300-500 |
| Processing Time | ~30-60 seconds |
| Feature Points Tracked | 50-300 per frame |
| Trajectory Points | Equal to frame count |
| Final Rotation | ~180° (return flight) |

### Accuracy Analysis

**Sources of Error:**
1. **Optical Flow Drift**: Accumulates 1-2 pixels per frame
2. **GSD Approximation**: Assumes constant altitude
3. **Lens Distortion**: Not corrected in this implementation
4. **GPS Conversion**: Simplified spherical Earth model

**Error Mitigation:**
- RANSAC outlier rejection
- Feature redetection when count drops
- Gaussian smoothing post-processing
- Median filtering of flow vectors

---

## Code Structure

### Main Processing Loop

```python
while True:
    # 1. Read next frame
    # 2. Calculate optical flow
    # 3. Estimate affine transformation
    # 4. Extract rotation and translation
    # 5. Transform to global coordinates
    # 6. Convert to GPS
    # 7. Update trajectory
    # 8. Refresh feature points if needed
```

### State Variables

- `x_m, y_m`: Local position in meters
- `theta`: Accumulated rotation in radians
- `lat, lon`: Current GPS position
- `trajectory`: List of all GPS points
- `prev_points`: Feature points from previous frame

---

## Visualization Output

The system generates an interactive HTML map (`drone_path.html`) with:

- **Blue Line**: Smoothed trajectory
- **Light Blue Line**: Raw trajectory
- **Green Marker**: Start position (field)
- **Red Marker**: Calculated end position
- **Orange Marker**: Ground truth end position (for validation)

Users can zoom, pan, and click markers for detailed information.

---

## Usage

```python
# Load video
cap = cv2.VideoCapture("video1.mp4")

# Set starting GPS coordinates
lat, lon = 48.2658, 25.9184

# Process video (automatic)
# ... (main loop runs)

# Output: drone_path.html
m.save("drone_path.html")
```
Predicted output: ![](https://github.com/skillfi/open-flow/blob/main/image.png)
Marks description:
* Green point: Start Position
* Red point: Predicted Position
* Orange Point: End position from video
---

## Limitations

1. **Altitude Dependency**: GSD must be recalculated for different altitudes
2. **No Vertical Tracking**: Assumes constant height flight
3. **Simplified GPS Model**: Uses flat-Earth approximations
4. **Cumulative Error**: Accuracy degrades over long flights
5. **No IMU Fusion**: Relies solely on visual data

---

## Future Improvements

### Accuracy Enhancements
- Integrate IMU data (gyroscope, accelerometer)
- Implement camera calibration for lens distortion
- Use full geodetic transformations (pyproj library)
- Add altitude tracking using structure-from-motion

### Robustness
- Implement loop closure detection
- Add GPS waypoint synchronization
- Handle video dropouts and occlusions
- Adaptive GSD based on detected altitude changes

### Performance
- GPU acceleration for optical flow
- Parallel frame processing
- Real-time streaming support

---

## Conclusion

This drone path tracking system successfully reconstructs flight trajectories from video using computer vision techniques. By combining optical flow tracking with rotation compensation and proper coordinate transformations, it achieves reasonable accuracy for visualization and analysis purposes. The approach is particularly effective for flights with significant rotation, such as the test case where the drone performed a 180° return maneuver.

**Key Achievement**: The system correctly handles drone rotation during flight, a critical requirement that simple translation-only tracking methods fail to address.

---

## References

- Lucas, B. D., & Kanade, T. (1981). "An Iterative Image Registration Technique"
- Shi, J., & Tomasi, C. (1994). "Good Features to Track"
- OpenCV Documentation: [Optical Flow](https://docs.opencv.org/4.x/d4/dee/tutorial_optical_flow.html)
- Earth Radius & GPS Conversion: [Geodetic Calculations](https://en.wikipedia.org/wiki/Geographic_coordinate_system)
