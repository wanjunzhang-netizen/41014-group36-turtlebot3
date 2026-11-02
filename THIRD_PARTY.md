# Sources and scope

- Official TurtleBot3: https://github.com/ROBOTIS-GIT/turtlebot3 ; pinned commit `fc817ce3073af1d6032397c64504134882af5e9a`. Burger URDF and STL assets are preserved under `models/turtlebot3_official/`, including original Apache-2.0 LICENSE and SOURCE.json.
- The imported model was adapted for MuJoCo: scene/root placement, wheel actuators and encoder sensors, caster friction/contact setup, LiDAR site and one added RGB camera with known mounting extrinsics. The original Burger URDF does not itself supply this RGB camera. `02_robot.xml` is the runtime model; unused intermediate XML has been removed.
- MuJoCo simulation/rendering/ray tools: https://mujoco.readthedocs.io/en/stable/python.html
- OpenCV ArUco detection and monocular PnP: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- Course basis: differential-drive exact arc odometry; Week 5 state-space; Week 6/8 feedback; Week 7/8 observer/EKF; Week 3 camera projection; Week 4 ranging; Week 9 visual guidance. Course PDFs/notebooks are not redistributed in this repository.
- Additional self-written engineering: mission state machine, short-waypoint clearance heuristic, random-room generation and generation-time grid connectivity check. Unknown landmark initialization uses a five-state joint EKF with a single-landmark EKF-SLAM structure. This is not a full environment SLAM map, A* planner or a global completeness guarantee.
- LiDAR least-squares scale/offset calibration and rotational-symmetry analysis are additional project validation work; the Lie-bracket comment is optional nonlinear-control context beyond the basic course rank test.
- Repository organization and code/documentation benefited from AI assistance; see CONTRIBUTIONS.md. No human code review or remote GitHub publication is claimed by this initial package.

Third-party asset licenses remain attached. No blanket new license is assigned to course-derived/student code by this packaging step; authors should choose an appropriate license only if permitted.

- Current monocular extension: four image corners and a known 0.18 m marker size are used with iterative solvePnP. Camera range/bearing are fused with encoders. LiDAR is avoidance-only. https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html

- Assignment alignment: A3 B.1 exact unicycle arc adapted to encoder increments; A3 A.1/A.2 discrete LQR straight-segment tracking (NumPy DARE iteration, same Bryson weights, robot-specific speed/time). A3 B.3 Joseph EKF structure is adapted to an unknown camera landmark. A2 ICP/SIFT odometry is not used by this monocular/avoidance pipeline. See 演示讲稿与作业来源.md for exact scope.
