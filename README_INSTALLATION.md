# AURA Robotics Lab — Backend Installation

This guide installs the **full upstream multi-UAV backend** used by the AURA Robotics Lab and prepares the local folder structure expected by the three labs.

The AURA Robotics Lab repository contains the **course files only**.  
The full simulator/backend is installed separately.

Official course repository:

https://github.com/aura-laboratory/AURA_ROBOTICS_LAB

---

# 1. System Requirements

The backend uses the MRS UAV System inside an **Apptainer** container.

Official Apptainer website:

https://apptainer.org/

Supported environments:

- **Linux**
- **Windows 11 with WSL2 running Linux**
- **Windows with a Linux virtual machine**
- **macOS with a Linux virtual machine**

All installation commands below must be executed **inside a Linux environment**.

## Recommended environment

The simplest option is an **Ubuntu-compatible Linux** system.

On Ubuntu-compatible systems, the upstream backend installer can install the required dependencies, install/configure Apptainer, download the pre-built container image, and compile the workspace.

## Non-Ubuntu Linux

Install Apptainer manually first, then continue with the installation below.

## Windows

Use either:

- Windows 11 + WSL2 + a Linux distribution, or
- a Linux virtual machine.

Run all commands from the Linux terminal.

## macOS

Use a Linux virtual machine and run all commands from inside that Linux environment.

## Disk space

The upstream backend requires approximately **6 GB** of disk space.

More free space is recommended for the course files, logs, results, and development tools.

---

# 2. Recommended Installation — Automatic Method

This is the recommended method.

## Step 1 — Clone the AURA Robotics Lab repository

Open a Linux terminal and run:

```bash
cd ~/Downloads

git clone https://github.com/aura-laboratory/AURA_ROBOTICS_LAB.git AURA_ROBOTICS_LAB
```

After cloning, you should have:

```text
~/Downloads/AURA_ROBOTICS_LAB/
```

## Step 2 — Run the automatic backend installer

```bash
cd ~/Downloads/AURA_ROBOTICS_LAB

chmod +x install_aura_backend.sh
./install_aura_backend.sh
```

The script automatically:

1. checks that it is running inside Linux,
2. checks that `git` is available,
3. creates the required AURA host folders,
4. creates:

```text
~/Downloads/aura-lab/
└── .aura_runtime/
    └── backend/
```

5. clones the full upstream backend,
6. checks out the backend version used by this course,
7. runs the upstream backend `install.sh`,
8. downloads/configures the required Apptainer-based runtime through the upstream installation procedure,
9. compiles/prepares the backend.

The backend is installed at:

```text
~/Downloads/aura-lab/.aura_runtime/backend
```

When the script finishes successfully, continue with the README of the lab you want to install.

---

# 3. Manual Installation — Alternative Method

Use this method if you prefer to perform every step manually.

## Step 1 — Clone the AURA Robotics Lab repository

```bash
cd ~/Downloads

git clone https://github.com/aura-laboratory/AURA_ROBOTICS_LAB.git AURA_ROBOTICS_LAB
```

## Step 2 — Create the AURA runtime folder

```bash
mkdir -p ~/Downloads/aura-lab/.aura_runtime
```

## Step 3 — Clone the full upstream backend

```bash
git clone https://github.com/ctu-mrs/summer-school-2026.git   ~/Downloads/aura-lab/.aura_runtime/backend
```

## Step 4 — Check out the course-tested backend version

The course package was prepared with backend commit:

```text
cff98a147c64316d5e3d30f61f954821c42685cf
```

Run:

```bash
cd ~/Downloads/aura-lab/.aura_runtime/backend

git checkout cff98a147c64316d5e3d30f61f954821c42685cf
```

## Step 5 — Run the upstream backend installer

```bash
cd ~/Downloads/aura-lab/.aura_runtime/backend

chmod +x install.sh
./install.sh
```

The upstream installer installs/sets up the required dependencies, downloads the Apptainer image, and compiles the workspace.

---

# 4. Install a Lab

After the backend installation is complete, install the lab you want to use.

The AURA repository contains:

```text
AURA_ROBOTICS_LAB/
├── lab1_motion_planning/
├── lab2_localization_slam/
└── lab3_decision_making/
```

Each lab folder contains its own:

- `README.md`
- installation script
- course files
- launch instructions

Follow the README inside the corresponding lab folder.

## Example — Lab 1

```bash
cd ~/Downloads/AURA_ROBOTICS_LAB/lab1_motion_planning

chmod +x install_lab1.sh
./install_lab1.sh ~/Downloads/aura-lab
```

Then:

```bash
cd ~/Downloads/aura-lab
```

Run one of the Lab 1 modes:

```bash
./run_lab1_rrt.sh
./run_lab1_astar.sh
./run_lab1_student.sh
```

For Lab 2 and Lab 3, follow the corresponding lab README.

---

# 5. Opening the Backend in PyCharm

The upstream backend provides a helper script for opening PyCharm inside the Apptainer environment.

Run:

```bash
cd ~/Downloads/aura-lab/.aura_runtime/backend

./simulation/pycharm.sh
```

The main AURA student-facing files are located under:

```text
mrim_task/mrim_planner/scripts/
```

The README of each lab explains exactly which file the student should edit.

---

# 6. Expected Folder Layout

After cloning the AURA repository and installing the backend:

```text
~/Downloads/
├── AURA_ROBOTICS_LAB/
│   ├── lab1_motion_planning/
│   ├── lab2_localization_slam/
│   ├── lab3_decision_making/
│   ├── README.md
│   ├── README_INSTALLATION.md
│   └── install_aura_backend.sh
│
└── aura-lab/
    └── .aura_runtime/
        └── backend/
```

After installing individual labs, their launcher scripts are copied into:

```text
~/Downloads/aura-lab/
```

For example:

```text
~/Downloads/aura-lab/
├── run_lab1.sh
├── run_lab1_rrt.sh
├── run_lab1_astar.sh
├── run_lab1_student.sh
├── run_lab2.sh
├── run_lab3.sh
└── .aura_runtime/
    └── backend/
```

Only the launchers for labs that have actually been installed will appear there.

---

# 7. Important Notes

- Do **not** clone the AURA course repository inside the backend folder.
- Do **not** manually install ROS on the host just for this course.
- Run all installation commands from a Linux environment.
- On Windows, use WSL2 or a Linux virtual machine.
- On macOS, use a Linux virtual machine.
- On non-Ubuntu Linux, install Apptainer manually before running the upstream installer.
- The recommended workflow is:
  1. clone `AURA_ROBOTICS_LAB`,
  2. run `install_aura_backend.sh`,
  3. install the desired lab,
  4. run the corresponding lab launcher.
