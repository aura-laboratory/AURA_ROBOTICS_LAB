# AURA Robotics Lab — Installation

This guide describes how to install the simulation backend used for the laboratory exercises of the Robotics course.

The `AURA_ROBOTICS_LAB` repository contains the course files only. The simulator and the main backend are installed separately.

Course repository:

https://github.com/aura-laboratory/AURA_ROBOTICS_LAB

---

## 1. System requirements

The simulator runs in Linux and uses the MRS UAV System through an Apptainer container.

The AURA Robotics Lab setup has been tested on:

- Ubuntu 20.04 LTS
- Ubuntu 22.04 LTS

For a new installation, **Ubuntu 20.04 LTS is recommended**.

The lab can also be used on:

- Windows 11 with WSL2 and Ubuntu
- Windows with an Ubuntu virtual machine
- macOS with a Linux virtual machine

All commands in this guide should be executed from a Linux terminal.

On Ubuntu, the installation scripts handle the required dependencies and the Apptainer setup automatically.

If another Linux distribution is used, Apptainer may need to be installed manually first.

The backend requires approximately **6 GB of disk space**. Some additional free space is recommended for course files, logs, results and development tools.

---

## 2. Automatic installation

This is the recommended installation method.

### Clone the course repository

Open a Linux terminal and run:

```bash
cd ~/Downloads

git clone https://github.com/aura-laboratory/AURA_ROBOTICS_LAB.git AURA_ROBOTICS_LAB
```

The repository should now be available at:

```text
~/Downloads/AURA_ROBOTICS_LAB
```

### Install the backend

Run:

```bash
cd ~/Downloads/AURA_ROBOTICS_LAB

chmod +x install_aura_backend.sh
./install_aura_backend.sh
```

The installation script creates the AURA runtime directory and installs the simulator backend under:

```text
~/Downloads/aura-lab/.aura_runtime/backend
```

During the installation, the script:

- checks the basic system requirements,
- creates the required folders,
- downloads the upstream backend,
- runs the upstream installation procedure,
- sets up the Apptainer environment,
- downloads the required container image,
- prepares and compiles the workspace.

When the installation finishes successfully, the backend is ready and the laboratory exercises can be installed.

---

## 3. Manual backend installation

The automatic installation method above is recommended.

If you prefer to install the backend manually, use the following steps.

### Create the runtime directory

```bash
mkdir -p ~/Downloads/aura-lab/.aura_runtime
```

### Clone the upstream backend

```bash
git clone https://github.com/ctu-mrs/summer-school-2026.git \
  ~/Downloads/aura-lab/.aura_runtime/backend
```

### Run the upstream installer

```bash
cd ~/Downloads/aura-lab/.aura_runtime/backend

chmod +x install.sh
./install.sh
```

The upstream installer sets up the required dependencies, Apptainer environment and simulator workspace.

---

## 4. Laboratory exercises

The course repository contains three laboratory folders:

```text
AURA_ROBOTICS_LAB/
├── lab1_motion_planning/
├── lab2_localization_slam/
└── lab3_decision_making/
```

Each laboratory has its own README with the relevant instructions.

### Lab 1 — Motion Planning

After the backend has been installed, install Lab 1 with:

```bash
cd ~/Downloads/AURA_ROBOTICS_LAB/lab1_motion_planning

chmod +x install_lab1.sh
./install_lab1.sh ~/Downloads/aura-lab
```

Then move to the AURA runtime directory:

```bash
cd ~/Downloads/aura-lab
```

The Lab 1 launchers are:

```bash
./run_lab1_rrt.sh
./run_lab1_astar.sh
./run_lab1_student.sh
```

The complete Lab 1 instructions are available in:

```text
~/Downloads/AURA_ROBOTICS_LAB/lab1_motion_planning/README.md
```

### Lab 2 — Localization / Estimation / SLAM

The installation and execution instructions for Lab 2 will be provided in:

```text
~/Downloads/AURA_ROBOTICS_LAB/lab2_localization_slam/
```

### Lab 3 — Decision Making / Reinforcement Learning

The installation and execution instructions for Lab 3 will be provided in:

```text
~/Downloads/AURA_ROBOTICS_LAB/lab3_decision_making/
```

---

## 5. Opening the backend in PyCharm

The backend includes a helper script for opening PyCharm from the Apptainer environment.

Run:

```bash
cd ~/Downloads/aura-lab/.aura_runtime/backend

./simulation/pycharm.sh
```

For Lab 1, the relevant planner files are located under:

```text
mrim_task/mrim_planner/
```

The Lab 1 README specifies which files should be inspected or modified.

---

## 6. Folder structure

After cloning the course repository and installing the backend, the expected structure is:

```text
~/Downloads/
├── AURA_ROBOTICS_LAB/
│   ├── lab1_motion_planning/
│   ├── lab2_localization_slam/
│   ├── lab3_decision_making/
│   ├── install_aura_backend.sh
│   ├── README.md
│   ├── README_INSTALLATION.md
│   ├── THIRD_PARTY_LICENSE_BSD-3-Clause.txt
│   └── THIRD_PARTY_NOTICE.md
│
└── aura-lab/
    └── .aura_runtime/
        └── backend/
```

After Lab 1 has been installed, its student-facing launchers are available under:

```text
~/Downloads/aura-lab/
├── run_lab1_rrt.sh
├── run_lab1_astar.sh
├── run_lab1_student.sh
└── .aura_runtime/
    └── backend/
```

Some internal helper scripts may also be present in this directory.

---

## 7. Notes

- Do not clone `AURA_ROBOTICS_LAB` inside the backend directory.
- Do not install ROS separately on the host for this course.
- Run the installation commands from a Linux environment.
- For a new installation, Ubuntu 22.04 LTS is recommended.
- On Windows 11, WSL2 with Ubuntu can be used.
- On macOS, use a Linux virtual machine.
- Follow the README of each laboratory before modifying backend files.

A typical installation sequence is:

```text
1. Clone AURA_ROBOTICS_LAB
2. Run install_aura_backend.sh
3. Open the README of the laboratory
4. Install the laboratory
5. Run its launcher
```
