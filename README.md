# Wadsworth

Wadsworth is a high-performance robotics side-quest project aimed at developing custom drivers and control software for the LeRobot SO-ARM101.

With this side-quest project I am to explore low-level hardware control and advanced robotics concepts, the ultimate goal of Wadsworth is to serve as the physical actuation layer for custom Vision-Language Models (VLMs). By mounting a camera to the arm (and eventually introducing a second arm), the goal is to enable precise navigation within the robot's kinematic workspace for complex pick-and-place tasks.

## Project Philosophy
* **Performance First:** Written in C++23 to execute as quickly and efficiently as possible.
* **Lightweight Architecture:** Intentionally designed with minimal dependencies and low overhead to ensure efficient execution on resource-constrained embedded systems.
* **Professional Conventions:** Enforces strict Google C++ and Python style guides via automated `pre-commit` hooks and GitHub Actions CI.
* **Portability:** Designed with a clean hardware abstraction layer, allowing current testing on Linux desktop with future plans to port to Zephyr RTOS.

## Roadmap
- [x] Project infrastructure, CI, and strict linting setup
- [x] Hardware Abstraction Layer (Serial Interface)
- [x] Linux Serial implementation
- [x] STS3215 Servo Protocol driver
- [x] Forward & Inverse Kinematics engine
- [ ] MuJoCo setup for simulation training
- [ ] VLM integration for visual workspace navigation
- [ ] Dual-arm synchronization

---

## Building from Source

The build system utilizes CMake and Ninja, and `uv` for isolated Python toolchain management.

### Prerequisites
You will need a C++23 compatible compiler, CMake, and Ninja:

```bash
sudo apt update && sudo apt install build-essential cmake ninja-build
```

#### Build and Run
```bash
git clone git@github.com:realPtolemy/wadsworth.git
cd wadsworth
mkdir build && cd build
cmake -G Ninja ..
ninja
./servo_test
```

#### For contributors
If you plan to write code or submit pull requests, please set up our automated linting pipeline (Google C++ and Python Style Guides).
Use uv to manage these tools:
```bash
curl -LsSf [https://astral.sh/uv/install.sh](https://astral.sh/uv/install.sh) | sh
uv sync
uv run pre-commit install
```
