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

After activating the environment, run a demo with one command:

```powershell
python main.py --seed 0
```

Other trials:

```powershell
python main.py
python main.py --seed 0 --camera-dropout 4
python main.py --seed 0 --lidar-sigma 0.012
```

No seed means a new random room. Click the dashboard and press Space or P to pause. Close the viewer normally to save results in outputs/monocular/. Repeated arguments overwrite the same output files. Headless tests still need OpenGL. Models are included; no ROS installation is needed.

## Files

| Location          | Purpose                                                   |
| ----------------- | --------------------------------------------------------- |
| main.py and src/  | Program entry point and simulation code.                  |
| models/           | Robot, room and licensed model assets.                    |
| calibration/      | Sensor settings and calibration checks.                   |
| tests/ and tools/ | Automated checks and trial evaluation.                    |
| evidence/         | Saved test summaries and dashboard image.                 |
| docs/design.md    | Model, frames, EKF, controller and parameters.            |
| docs/results.md   | Measured results and limits of the tests.                 |
| website/          | Project page source, published to the github_page branch. |
| requirements.txt  | Python packages needed to run the program.                |

## Demonstration video

The video shows normal docking and the estimate, uncertainty and true path.

## Individual contribution

wanjun zhang is the only student member and accounts for 100% of the team contribution: task requirements, simulation review and testing, design decisions, final submission and explanation.

**Generative AI declaration:** Codex assisted with implementation, debugging, documentation and automated checks. The team share does not mean the work was done without AI assistance. No peer review or real-hardware validation is claimed.
