# Group 36: TurtleBot3 visual docking

Track A. Simulation only, using MuJoCo.

**Student:** wanjun zhang, 25202185. Sole student member.

- [Code](https://github.com/wanjunzhang-netizen/41014-group36-turtlebot3)
- [Project page](https://wanjunzhang-netizen.github.io/41014-group36-turtlebot3/)
- [Demonstration video](https://wanjunzhang-netizen.github.io/41014-group36-turtlebot3/#demo)
- [Design and parameter values](docs/design.md)
- [Validation results](docs/results.md)

The robot uses one RGB camera to find a known-size ArUco marker. An EKF combines camera range and bearing with wheel odometry. LiDAR is used only to avoid obstacles. An LQR tracker uses the estimated pose. The goal is to face the marker and put a wheel inside the charging circle.

## Install and run

Use Windows, Python 3.11 and a working OpenGL graphics driver. Open a terminal in this repository:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe main.py --seed 0
```

Other trials:

```powershell
python main.py
python main.py --seed 0 --camera-dropout 4
python main.py --seed 0 --lidar-sigma 0.012
```

No seed gives a new random room. Space/P pauses the dashboard. Results are saved in outputs/monocular/.

## Demonstration video

The video shows normal docking and the estimate, uncertainty and true path.

## Individual contribution

wanjun zhang is the only student member and accounts for 100% of the team contribution: task requirements, simulation review and testing, design decisions, final submission and explanation.

**Generative AI declaration:** Codex assisted with implementation, debugging, documentation and automated checks.

Course code adapted from Assessment 3: motion model, Joseph EKF update and LQR. Third-party sources and licenses: [THIRD_PARTY.md](THIRD_PARTY.md).
