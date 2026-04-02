Chapter 2: Proposed Design

2.1 System Architecture Overview
The proposed Advanced Rider Assistance System is designed to enhance rider safety through comprehensive situational awareness and proactive threat detection. The architecture follows a modular, sense-plan-act paradigm. The sensing layer comprises a monocular camera and a millimeter-wave radar. The perception and planning layer executes coordinate transformation, sensor fusion, multi-object tracking, and collision threat assessment. The action layer consists of haptic and audio actuators for rider alerts and a remote graphical interface for real-time visualization.
[Insert Image Here: Block diagram illustrating the system architecture, showing data flow from Camera and Radar to the fusion module, tracker, and finally to the visualization and haptic outputs]

2.2 Sensor Modalities and Justification
The system employs a complementary sensor suite to overcome the limitations of individual modalities. The monocular camera provides high-resolution semantic information, enabling the classification of dynamic obstacles such as cars, trucks, and pedestrians, as well as static infrastructure like traffic signs. However, monocular vision struggles with accurate depth estimation. To address this, an mmWave radar is integrated. Radar excels at direct distance and relative velocity measurement, remaining robust under adverse weather and lighting conditions. 

2.3 Coordinate Transformation and Inverse Perspective Mapping (IPM)
A fundamental challenge in sensor fusion is aligning data from disparate sensors which "see" the world differently into a common reference frame. The camera inherently captures a two-dimensional perspective projection of the three-dimensional world, meaning objects appear smaller as they get further away, and distances are difficult to gauge directly from pixels. The radar, conversely, provides direct metric distances.

To bridge this gap, the proposed design utilizes a two-dimensional metric Bird's Eye View (BEV) ground plane as the unified coordinate system. We must transform the camera's pixel coordinates $(u, v)$ into this top-down metric view $(x, y)$. This process is known as Inverse Perspective Mapping (IPM) and is achieved using Homography.

Assuming the road surface is relatively flat, the geometric relationship between the camera's image plane and the ground plane can be mapped using a $3 \times 3$ transformation matrix called a Homography matrix ($H$). 

The transformation process begins by identifying the bottom-center pixel of a detected bounding box, denoted as $p_{img} = [u, v, 1]^T$. This specific point is chosen because it represents where the object (e.g., a car's tire) contacts the ground plane.

This pixel coordinate is then multiplied by the calibrated Homography matrix to project it into the 3D world space:

$$
\begin{bmatrix} x' \\ y' \\ w \end{bmatrix} = H \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
$$

Because this is a projective transformation, the resulting coordinates are in a homogenous format (meaning they are scaled by a factor $w$). To retrieve the actual physical distances in meters on the flat ground plane, we normalize the coordinates by dividing by $w$:

$$
X = \frac{x'}{w}, \quad Y = \frac{y'}{w}
$$

Here, $X$ represents the lateral distance (left/right) from the ego-vehicle, and $Y$ represents the longitudinal distance (straight ahead). By applying this mathematical projection, the system successfully translates a flat 2D image into a measurable metric space, allowing camera detections to be directly compared and fused with radar distance readings.

2.4 Sensor Fusion Strategy
The fusion strategy employs a two-level hierarchical approach to maximize detection reliability while maintaining a broad field of view. Level 1 fusion associates camera bounding boxes with radar targets based on spatial proximity in the Bird's Eye View plane. Objects confirmed by both sensors are assigned a high confidence score, leveraging the radar's precise distance and the camera's classification. Level 2 processing handles camera-only detections. These objects, which may fall outside the radar's field of view or lack sufficient radar cross-section, are still tracked but assigned a lower confidence penalty. Traffic signs are processed via an independent vision-only pipeline, as they do not require radar validation for basic positional logging.

2.5 Multi-Object Tracking and Kalman Filtering
To transition from isolated, frame-by-frame detections to persistent situational awareness, a Multi-Object Tracking system is implemented. Raw sensor data is often noisy, detections can occasionally drop out due to occlusion, and objects need to be continuously tracked over time to calculate their speed and direction. To address this, the system employs a Constant Velocity Kalman Filter for each detected object.

The Kalman Filter is a recursive algorithm that provides an optimal estimate of an object's true state by smoothing out measurement noise. For this system, the "state" of an object is defined by its 2D position and its velocity. The state vector at any given time step $k$ is represented as:

$$x_k = \begin{bmatrix} x \\ y \\ v_x \\ v_y \end{bmatrix}$$

The Kalman Filter operates in a continuous two-step loop: **Prediction** and **Update**.

**1. The Prediction Step:**
Before receiving a new sensor reading, the filter predicts where the object should be based on its last known state and the assumption that it is moving at a constant velocity. 

$$x_{k|k-1} = F x_{k-1|k-1}$$
$$P_{k|k-1} = F P_{k-1|k-1} F^T + Q$$

Here, $x_{k|k-1}$ is the predicted new state. $F$ is the state transition matrix, which physically models the constant velocity assumption over a time step $\Delta t$:

$$F = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

$P$ represents the covariance (the uncertainty or error in the estimate), and $Q$ introduces process noise to account for the fact that real objects don't move at a perfectly constant velocity (they accelerate and brake).

**2. The Update Step:**
When a new sensor measurement arrives, the filter updates its prediction. The raw sensor only provides position $(x, y)$, not direct velocity, so the measurement $z_k$ is mapped using an observation matrix $H$:

$$z_k = \begin{bmatrix} x \\ y \end{bmatrix}, \quad H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

The filter calculates the difference between the actual measurement and the predicted measurement ($y_k$, the "innovation") and uses the Kalman Gain ($K_k$) to decide how much to trust the new measurement versus the old prediction:

$$y_k = z_k - H x_{k|k-1}$$
$$S_k = H P_{k|k-1} H^T + R$$
$$K_k = P_{k|k-1} H^T S_k^{-1}$$

Finally, the state estimate and uncertainty are updated:

$$x_{k|k} = x_{k|k-1} + K_k y_k$$
$$P_{k|k} = (I - K_k H) P_{k|k-1}$$

Where $R$ is the measurement noise (how noisy we expect the sensor to be). When radar data is available and fused, the highly accurate doppler velocity directly updates the $v_y$ state, drastically improving the filter's accuracy.

**Data Association:**
At every frame, the new list of detections must be assigned to the existing Kalman Filter tracks. We solve this using the **Hungarian Algorithm** (Linear Sum Assignment). The goal is to minimize the total global distance between where tracks were predicted to be and where the new detections actually are.

Let $T$ be the set of existing tracks and $D$ be the set of new detections. A cost matrix $C$ is constructed using the Euclidean distance:

$$C_{i,j} = \sqrt{(x_{T_i} - x_{D_j})^2 + (y_{T_i} - y_{D_j})^2}$$

The Hungarian algorithm finds the optimal binary assignment that minimizes the total cost. Matches exceeding a physical threshold (e.g., 2.0 meters) are rejected to prevent tracks from jumping erratically to distant objects.

2.6 Time-To-Collision (TTC) and Multi-Modal Warning Actuation
The ultimate objective of the perception stack is to identify impending collisions and trigger preventative action. Rather than relying on simple static distance thresholds (which would trigger unnecessary alerts in slow traffic and be too late on highways), the system utilizes **Time-To-Collision (TTC)**. TTC accounts for the closing speed between the ego-vehicle and the tracked object.

For a tracked object located at a longitudinal distance $y$ directly in front of the motorcycle (within the lane corridor boundaries, typically $|x| < \text{LANE\_HALF\_WIDTH}$), and possessing a relative longitudinal approach velocity $v_y$ (where $v_y < 0$ means it is getting closer), the TTC is calculated as:

$$TTC = \frac{|y|}{|v_y|}$$

If the calculated $TTC$ falls below a critical safety threshold (e.g., $TTC < 2.0$ seconds), the system generates a collision alert.

[Insert Image Here: Diagram illustrating the Time-To-Collision concept, showing a vehicle ahead with a relative velocity vector pointing towards the ego-vehicle]

**Haptic and Audio Feedback:**
The warning mechanism utilizes a multi-modal approach combining variable-intensity haptic feedback and audio alerts. To provide intuitive warnings, the intensity of the haptic vibrators scales dynamically based on the urgency of the threat and the system's confidence in the tracking data. 

For frontal threats, the haptic intensity is calculated as:

$$\text{Intensity} = \min\left(1.0, \text{Confidence} \times \frac{TTC\_THRESHOLD}{\max(TTC, 0.1)}\right)$$

This formula ensures that as the TTC decreases (the threat becomes more imminent), the physical vibration intensity ramps up to a maximum of $1.0$ (100% power). Simultaneously, an audio speaker outputs a distinct warning tone for frontal collisions to command immediate attention. Lateral threats trigger directional vibrations on the corresponding side (left or right handlebar grips), providing spatial awareness without overwhelming the rider with audio alarms.

**Auto-Braking Safety Logic:**
In scenarios where haptic warnings are insufficient, the system is designed to interface with a physical brake actuator. However, autonomous braking on a two-wheeler carries inherent dynamic risks. Therefore, the auto-braking is governed by a strict operational envelope tied to the ego-vehicle's speed (measured via an onboard GPS or wheel speed sensor).

The actuator will only fire if the ego-vehicle is traveling within a safe speed window:

$$5.0 \text{ km/h} \leq \text{Ego Speed} \leq 50.0 \text{ km/h}$$

This critical safety interlock prevents the system from suddenly locking the brakes at dangerous highway speeds (which could cause a loss of control) or constantly actuating while maneuvering slowly through tight environments like parking lots.


Chapter 3: Implementation, Testing and Validation

3.1 Software Implementation and Concurrency
The system is implemented using an asynchronous programming model to handle the disparate update rates of the sensors. The camera and radar data acquisition routines operate concurrently, feeding a central processing loop. This asynchronous design prevents slower sensors or computationally heavy vision models from blocking the processing of high-frequency data, ensuring low-latency threat detection.

3.2 Camera Calibration and Inverse Perspective Mapping
The transition from theory to practice required rigorous camera calibration. In the implemented Python codebase (specifically within the `BEV.py` module), this was executed in two distinct phases: intrinsic calibration and extrinsic ground-plane projection.

**1. Intrinsic Calibration (Lens Distortion & Camera Matrix):**
Before any distance can be accurately measured, the camera's unique optical properties must be quantified. We implemented a calibration script using OpenCV's `calibrateCamera` function. This involved capturing multiple images of a standard physical checkerboard pattern (where the exact physical size of each square is defined globally as `CALIB_SQUARE_SIZE_M = 0.025` meters) from various angles. 

To map the mathematical setup, a set of 3D object points $P_{obj}$ is generated for a checkerboard with columns and rows defined by `CALIB_BOARD_SIZE = (9, 6)`:
$$P_{obj} = \begin{bmatrix} 0 & 0 & 0 \\ L & 0 & 0 \\ \vdots & \vdots & \vdots \\ (C-1)L & (R-1)L & 0 \end{bmatrix}$$

By matching these known 3D points to their detected 2D pixel coordinates across all images, the algorithm computes the optimal camera intrinsic matrix ($K$) which models the focal lengths ($f_x, f_y$) and optical centers ($c_x, c_y$):
$$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$

Simultaneously, it calculates a $1 \times 5$ array of distortion coefficients ($D$) to correct for radial ($k_1, k_2, k_3$) and tangential ($p_1, p_2$) lens distortion:
$$D = \begin{bmatrix} k_1 & k_2 & p_1 & p_2 & k_3 \end{bmatrix}$$
This step is crucial because cheap monocular cameras often have significant "fisheye" distortion at the edges of the frame, which would warp our distance estimates.

**2. Extrinsic Calibration (The BEV Ground Plane):**
Once the lens is corrected via $K$ and $D$, we configure the Homography matrix ($H$) to map pixels to the flat ground. In the implementation, this is done by manually identifying four pixel coordinates in the camera frame ($u_i, v_i$) that correspond to a known rectangular area on the physical ground, yielding world coordinates ($X_i, Y_i$) in meters. 

For example, a $3m \times 5m$ grid marked on the pavement relative to the camera origin might yield the following world points matrix $P_{world}$:
$$P_{world} = \begin{bmatrix} -1.5 & 2.5 \\ 1.5 & 2.5 \\ 1.5 & -2.5 \\ -1.5 & -2.5 \end{bmatrix} \text{ (Top-Left, Top-Right, Bottom-Right, Bottom-Left)}$$

This must be directly mapped to a corresponding matrix of pixel coordinates $P_{image}$ extracted from the camera frame:
$$P_{image} = \begin{bmatrix} u_1 & v_1 \\ u_2 & v_2 \\ u_3 & v_3 \\ u_4 & v_4 \end{bmatrix}$$

By passing both $P_{world}$ and $P_{image}$ into OpenCV's `findHomography` function, the system solves the linear system to generate the $3 \times 3$ transformation matrix used during runtime to convert the bottom-center of bounding boxes into metric $(x, y)$ coordinates.

3.3 Multi-Object Tracking and Kalman Filter Realization
The Multi-Object Tracking pipeline was realized in `tracker.py` by instantiating individual `Track` objects that encapsulate their respective Kalman filters. 

While the theoretical Kalman Filter involves dynamically updating covariance based on complex system modeling, the actual software implementation simplifies this for real-time performance on constrained hardware. We implemented a linear Constant Velocity model using NumPy matrices operating at a fixed time step defined by `KALMAN_DT = 0.05` seconds, representing a $20$ Hz update loop.

Upon instantiating a new `Track` object, the initial state vector $x_0$ and the state covariance matrix $P_0$ are defined. The initial velocity is derived from the radar's Doppler speed if available, otherwise defaulting to zero:
$$x_0 = \begin{bmatrix} x_{det} \\ y_{det} \\ 0 \\ v_{y, \text{radar}} \end{bmatrix}, \quad P_0 = 10.0 \times \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
The scalar multiplier $10.0$ reflects a high initial uncertainty before the filter has processed subsequent frames.

The State Transition Matrix ($F$) is implemented as:
$$F = \begin{bmatrix} 1 & 0 & 0.05 & 0 \\ 0 & 1 & 0 & 0.05 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

The process noise covariance ($Q$) and measurement noise covariance ($R$) matrices were initialized as static, pre-tuned matrices rather than constantly recalculating them based on instantaneous sensor variance. We applied a process noise magnitude `KALMAN_PROCESS_NOISE_Q = 0.5` applied to a discrete-time continuous-white-noise acceleration model:
$$Q = q \begin{bmatrix} \frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} & 0 \\ 0 & \frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} \\ \frac{\Delta t^3}{2} & 0 & \Delta t^2 & 0 \\ 0 & \frac{\Delta t^3}{2} & 0 & \Delta t^2 \end{bmatrix}$$

The measurement noise $R$ was set to a constant metric defined by `KALMAN_MEASURE_NOISE_R` to represent expected baseline spatial variance from the sensors:
$$R = \begin{bmatrix} \sigma_{x}^2 & 0 \\ 0 & \sigma_{y}^2 \end{bmatrix}$$

The data association logic (Hungarian algorithm) was implemented using SciPy's `linear_sum_assignment` function applied to a cost matrix populated by the Euclidean distances between predicted track states and new detections. A crucial gating threshold was added: any mathematically optimal match that exceeded a physical distance defined by `TRACK_MAX_DISTANCE` was aggressively rejected. This prevents tracks from erratically jumping across the screen when a detection is temporarily dropped and a new, unrelated object appears elsewhere. Furthermore, tracks require `TRACK_CONFIRM_HITS` consecutive hits to be upgraded from "tentative" to "confirmed", and are deleted if missed for `TRACK_DELETE_MISSES` consecutive frames, ensuring robustness against flickering detections.

3.4 Sensor Fusion and Threat Assessment Logic
The sensor fusion logic implemented in `system.py` executes sequentially at every frame. First, camera detections are projected to the BEV plane. Then, radar targets (which natively arrive in polar coordinates of angle and distance) are converted to Cartesian $(x, y)$ coordinates.

The system utilizes a spatial-proximity matching loop. If a camera bounding box center falls within `FUSION_MATCH_DIST` meters of a radar target, they are considered "fused". Fused objects immediately inherit the highly accurate Doppler speed directly from the radar, and their tracking confidence is multiplied by `RADAR_CONFIDENCE_BOOST`. If no radar match is found, the object is marked as "camera-only" and assigned a lower base confidence score defined by `CAM_ONLY_CONFIDENCE`.

Crucially, the theoretical Kalman velocity ($v_y$) is enhanced using this fused radar data. When radar speed is available, the tracker employs an Alpha filter (with `RADAR_VELOCITY_ALPHA`) to nudge the Kalman Filter's internal velocity estimate toward the direct radar measurement:
$$v_{y, \text{new}} = (1 - \alpha) \cdot v_{y, \text{kalman}} + \alpha \cdot v_{y, \text{radar}}$$
This ensures the tracked velocity remains highly accurate even if the bounding box slightly jitters frame-to-frame.

The collision detection utilizes the Time-To-Collision (TTC) metric as defined in Chapter 2. The collision loops evaluate every confirmed track within the immediate front corridor defined by `LANE_HALF_WIDTH` that has a negative relative velocity (approaching the ego-vehicle). If the TTC drops below the `TTC_THRESHOLD` second threshold, the warning flags are raised, and the haptic intensity is calculated dynamically before being dispatched to the actuator controller over the local network.

3.5 Visualization Interface Development and User Interaction
To facilitate testing and demonstrate system capabilities, a real-time three-dimensional visualization interface was developed. Built using web technologies (HTML, CSS, JavaScript) and the Three.js library, the interface connects to the core Python system via a WebSocket server operating on a local network port. This architecture allows for the decoupled rendering of the vehicle's environment on any device equipped with a modern web browser.
[Insert Image Here: Screenshot of the 3D web visualization interface, showing tracked vehicles as 3D bounding boxes, traffic signs, and the collision warning UI overlay]

The visualization interface provides the user with a comprehensive, interactive dashboard to monitor the system's perception state. The user interactions and visual components are detailed as follows:

1. 3D Environment Navigation: Upon opening the interface, the user sees a central green marker representing the ego-vehicle on a 3D grid floor. The user can actively explore this environment using standard mouse controls. Left-clicking and dragging rotates the camera perspective around the ego-vehicle, scrolling the mouse wheel zooms the view in and out for macro or micro situational awareness, and right-clicking allows panning across the scene.

2. Connection Status Monitoring: At the top right of the interface, a persistent pill-shaped indicator informs the user of the WebSocket connection status, glowing green for "CONNECTED" and red for "DISCONNECTED". The system automatically attempts to reconnect if the connection drops.

3. Real-time Object Tracking: Tracked objects are rendered as 3D bounding boxes. The user can visually identify the object class by the box's color (e.g., cars are green, trucks are orange, pedestrians are yellow) and dimensions. The system visually differentiates between 'confirmed' and 'tentative' tracks. Confirmed tracks are rendered with high opacity, while tentative tracks are semi-transparent, allowing the user to intuitively gauge the system's confidence.

4. Static Infrastructure Observation: Traffic signs are rendered uniquely as 3D poles topped with color-coded signboards (e.g., red for stop signs). This allows the user to see static environmental features that the camera has detected.

5. Diagnostic Information Panel: On the left side of the screen, a fixed overlay panel provides the user with quantitative data. The user can monitor the total count of confirmed and tentative objects, and the overall system update rate in Hertz. Below this, a dynamic list updates in real-time with the details of all currently tracked objects and signs. For each object, the user can read its unique ID, semantic class, exact (x, y) coordinates in meters, estimated velocity in meters per second, and a confidence score.

6. Interacting with Collision Warnings: The interface is designed to alert the user immediately of impending threats. When an object enters the critical frontal threat zone, a large, flashing red "COLLISION WARNING" banner appears in the center of the user's screen, representing the distinct audio warning tone triggered by the physical system. A contextual information box beneath it tells the user the exact distance to the obstacle. Furthermore, three circular indicators at the bottom of the screen (labeled LEFT, FRONT, RIGHT) simulate the physical haptic feedback. If a threat is detected on a specific side, the corresponding indicator flashes red and vibrates on screen, allowing the user to visually verify the haptic actuator commands.

3.6 Validation Methodology and Parameter Tuning
Validation of the prototype focused on functional correctness, algorithm stability, and the empirical fine-tuning of system parameters. The testing environment initially utilized simulated data streams to mimic complex traffic scenarios, allowing for repeatable evaluation of the tracking and fusion logic without the immediate risks of on-road testing. This was followed by controlled real-world validation.

A critical phase of the validation methodology involved tuning the global parameters defined in the system architecture to achieve optimal performance:
1. **Kalman Noise Covariance Tuning:** The process noise (`KALMAN_PROCESS_NOISE_Q`) and measurement noise (`KALMAN_MEASURE_NOISE_R`) were adjusted to balance tracking smoothness against responsiveness.
2. **Track Lifecycle Tuning:** The thresholds for confirming (`TRACK_CONFIRM_HITS`) and deleting (`TRACK_DELETE_MISSES`) tracks were tuned to minimize ghost detections without introducing dangerous latency.
3. **Fusion Spatial Thresholds:** The maximum allowable distance for camera-radar fusion (`FUSION_MATCH_DIST`) and track association (`TRACK_MAX_DISTANCE`) were empirically determined based on sensor alignment variance.
4. **Threat Assessment Bounds:** The time-to-collision threshold (`TTC_THRESHOLD`) and corridor width (`LANE_HALF_WIDTH`) were tuned to provide adequate warning time while minimizing false positives from adjacent lanes.

Key validation metrics included:
A. Coordinate Projection Accuracy: Verifying that camera detections at known pixel coordinates accurately mapped to the correct real-world distances.
B. Tracking Stability: Observing the frequency of ID switches and track fragmentation when objects crossed paths or were temporarily lost by the simulated sensors.
C. Warning Latency: Ensuring that the collision detection logic triggered the appropriate haptic commands, audio alerts, and UI warnings immediately upon an object entering a defined threat zone.

3.7 Validation Results, Parameter Finalization, and Analysis
Through iterative testing, the global system parameters were finalized to the following values, which provided the most stable performance for the prototype:

**Final Track Management Parameters:**
*   `KALMAN_PROCESS_NOISE_Q = 0.5`: Provided sufficient flexibility for accelerating targets.
*   `KALMAN_MEASURE_NOISE_R = 0.3` meters: Accurately reflected the baseline spatial variance of the sensors.
*   `TRACK_MAX_DISTANCE = 2.0` meters: Prevented errant ID switches during temporary occlusions.
*   `TRACK_CONFIRM_HITS = 3` (frames): Ensured a track was legitimate (taking $0.15$ seconds at $20$Hz) before processing it for collision.
*   `TRACK_DELETE_MISSES = 5` (frames): Allowed a track to survive a brief $0.25$-second occlusion.

**Final Fusion and Threat Parameters:**
*   `FUSION_MATCH_DIST = 1.0` meters: Successfully paired camera and radar detections without merging adjacent distinct objects.
*   `RADAR_VELOCITY_ALPHA = 0.3`: A $30\%$ radar velocity injection per frame smoothed the Kalman estimate perfectly without causing erratic velocity jumps.
*   `LANE_HALF_WIDTH = 1.0` meters: Created a $2.0$-meter wide threat corridor, matching a standard motorcycle lane position.
*   `TTC_THRESHOLD = 2.0` seconds: Provided adequate time for human reaction and mechanical braking.

With these parameters locked, the tracking system effectively smoothed noisy trajectory data and maintained object identities through brief periods of simulated occlusion. The two-level fusion strategy proved valuable; the system reliably prioritized radar-confirmed targets (boosting confidence by `RADAR_CONFIDENCE_BOOST = 1.2`) for imminent collision warnings while maintaining a broader awareness of the environment through camera-only tracks (base confidence `CAM_ONLY_CONFIDENCE = 0.4`). The asynchronous architecture successfully maintained a high update rate for the visualization and warning logic, decoupled from the simulated sensor read times.


Chapter 4: Reflection on Prototype

4.1 Capabilities and Strengths
The developed Advanced Rider Assistance System prototype successfully demonstrates the feasibility of combining low-cost sensors to achieve sophisticated situational awareness for two-wheeled vehicles. The primary strength of the prototype lies in its robust sensor fusion and temporal tracking architecture. By synthesizing the semantic richness of vision with the spatial reliability of radar, the system mitigates the inherent weaknesses of each individual modality.
The asynchronous software architecture also proved highly effective, ensuring that the critical path for collision detection maintained a high operational frequency regardless of visualization or heavy vision processing overhead.

4.2 Limitations and Technical Challenges
Despite its successes, the current prototype exhibits several limitations that present opportunities for refinement. The reliance on a flat-ground assumption for the homography projection remains a vulnerability. In real-world scenarios involving significant road gradients, speed bumps, or uneven terrain, the geometric projection from pixel to world coordinates will introduce errors, potentially affecting the accuracy of distance estimations for camera-only detections.
Additionally, the Hungarian algorithm, while effective for basic scenarios, struggles in highly dense environments where multiple targets are clustered closely together. The current Euclidean distance cost metric does not account for the visual appearance or velocity profiles of the targets during the association phase, which can lead to identity swaps in complex traffic conditions.
The mmWave radar also presents resolution challenges. Discriminating between closely spaced objects, such as two pedestrians walking side-by-side, can be difficult, occasionally causing the fusion module to incorrectly merge distinct targets.

4.3 Reflection on Inertial Measurement Unit Integration
Although ultimately excluded from the final deployed prototype due to hardware constraints and consensus during the development phase, significant theoretical work was invested in integrating an Inertial Measurement Unit (IMU) for dynamic tilt correction during the Inverse Perspective Mapping process. 

Motorcycles and bicycles, unlike four-wheeled vehicles, lean significantly into corners. This roll angle drastically alters the camera's perspective relative to the ground plane. When the bike leans, the static Homography assumption (that the camera is perfectly level with the ground) is violated, leading to severe errors in estimating the distance to objects.

To dynamically "un-tilt" the camera pixels caused by the bike leaning or accelerating, we developed a mathematical framework to compute a per-frame rotation matrix $R$ derived from the IMU's accelerometer gravity vector. 

First, the pitch and roll angles are calculated from the accelerometer data ($a_x, a_y, a_z$):

**Roll Angle ($\theta_{roll}$):**
$$ \theta_{roll} = \arctan2(a_y, a_z) $$

**Pitch Angle ($\theta_{pitch}$):**
$$ \theta_{pitch} = \arctan2(-a_x, a_z) $$

These dynamic angles define a rotation matrix:
$$R = R_x(\theta_{pitch}) R_y(\theta_{roll})$$

This rotation matrix is then combined with the camera's intrinsic matrix $K$ to create a dynamic tilt homography $H_{tilt}$:
$$H_{tilt} = K R K^{-1}$$

In theory, before applying the base ground-plane Homography $H_{ground}$, the system would first project the bounding box's pixel $p_{img}$ through $H_{tilt}$ to simulate a perfectly level camera:
$$p_{level} = H_{tilt} \cdot p_{img}$$
$$p_{world} = H_{ground} \cdot p_{level}$$

While not implemented in the current iteration, this theoretical foundation remains a critical area for future development. True metric accuracy from a monocular camera on a leaning two-wheeler fundamentally requires this level of dynamic pose compensation.

4.4 Future Work and Evolution
Moving forward, several enhancements are proposed to elevate the prototype to a production-ready system. First, the spatial projection mechanism should be upgraded from a flat-plane homography approach to a full three-dimensional depth estimation model, potentially utilizing stereo vision or deep learning-based monocular depth estimation. This would eliminate the vulnerabilities associated with uneven terrain.
Second, the data association logic within the tracking module should be expanded to incorporate methodologies which utilize bounding box visual feature extraction to re-identify objects, drastically reducing ID switches during prolonged occlusions.
Finally, extensive real-world, on-road testing is paramount. Future iterations must involve empirical data collection under varied environmental conditions, including rain, fog, and nighttime riding. User experience studies will also be crucial to fine-tune the multi-modal haptic and audio feedback mapping, ensuring that warnings are perceivable, intuitive, and not overly intrusive to the rider's primary task of vehicle control.