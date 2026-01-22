# AAE5303 Environment Setup Report

> **Report Generated:** January 21, 2026  
> **Based on:** [PolyU-AAE5303-assignment-1-template](https://github.com/qmohsu/PolyU-AAE5303-assignment-1-template)

---

## 1. System Information

**Laptop model:**  
Lenovo XiaoXin14 Pro IHU 2021

**CPU / RAM:**  
Intel Core i5-11320H @ 3.20GHz, 7.6GB RAM

**Host OS:**  
Windows 11 (host) + Ubuntu 22.04.5 LTS (WSL2, kernel `Linux-6.6.87.2-microsoft-standard-WSL2-x86_64-with-glibc2.35`)


**Linux/ROS environment type:**  
- [ ] Dual-boot Ubuntu
- [x] WSL2 Ubuntu
- [ ] Ubuntu in VM (UTM/VirtualBox/VMware/Parallels)
- [ ] Docker container
- [ ] Lab PC
- [ ] Remote Linux server

**Linux distribution details:**
- Distribution: Ubuntu 22.04.5 LTS (Jammy)
- Kernel: Linux 6.6.87.2-microsoft-standard-WSL2
- Python version: 3.10.12

---

## 2. Python Environment Check

### 2.1 Steps Taken

This section describes how to create and activate the Python environment.

**Tool used:**  
Initially attempted to use venv, but due to missing dependencies, switched to user-level pip installation

**Key commands I ran:**

```bash
# Attempted to create virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install dependencies (need to resolve venv issue first or use system Python)
pip3 install --user -r requirements.txt
```

**Any deviations from the default instructions:**  
Encountered missing `python3-venv` package issue. Need to install this package before creating virtual environment.

### 2.2 Test Results

Run these commands and paste the actual terminal output (not just screenshots):

```bash
python3 scripts/test_python_env.py
```

**Output:**
```
==================================================
Python Environment Test Starting
==================================================
Operating System: Linux 6.6.87.2-microsoft-standard-WSL2

Python version: 3.10.12 (main, Jan  8 2026, 06:52:19) [GCC 11.4.0]
Python path: /usr/bin/python3
✓ Python version check passed

✓ NumPy version: 2.2.6
✓ NumPy functionality test passed
✓ Open3D version: 0.19.0
✓ Matplotlib version: 3.10.8

==================================================
✓ All tests passed! Python environment configured correctly.
==================================================
```

```bash
python scripts/test_open3d_pointcloud.py
```

**Output:**
```
==================================================
Open3D Point Cloud Test Starting
==================================================

Creating test point cloud...
✓ Successfully created point cloud with 1008 points
  Point cloud center: [0.49602837 0.48802747 0.49383611]
  Point cloud range: min=[0. 0. 0.], max=[1. 1. 1.]
✓ Points after downsampling: 826
✓ Successfully estimated normals

==================================================
Note: Running in WSL/headless environment,
visualization window may not display.
But Open3D library functionality tests passed.
==================================================

Attempting to open visualization window...
If you're in a remote or WSL environment, the window may not display.
This is normal, tests have passed.
✓ Visualization functionality available (actual display skipped)

==================================================
✓ Open3D point cloud test passed!
==================================================
```

**Screenshot:**  
<img width="2880" height="1704" alt="image" src="https://github.com/user-attachments/assets/9bd2cd88-cd05-4efe-8594-0d4e1ef4d60f" />


<img width="2880" height="1704" alt="image" src="https://github.com/user-attachments/assets/77cff7fd-d8d4-45a3-ba8d-445527715c58" />


---

## 3. ROS 2 Workspace Check

### 3.1 Build the workspace

Paste the build output summary (final lines only):

```bash
source /opt/ros/humble/setup.bash
colcon build
```

**Expected output:**
```
Summary: 1 package finished [x.xx s]
```

**Your actual output:**
```
Starting >>> env_check_pkg
Finished <<< env_check_pkg [0.90s]
Summary: 1 package finished [1.21s]
```

### 3.2 Run talker and listener

Show both source commands:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Then run talker:**
```bash
ros2 run env_check_pkg talker.py
```

**Output (3–4 lines):**
```
[INFO] [1769095846.033966906] [talker]: Talker node started
[INFO] [1769095847.026510174] [talker]: Publishing: "Hello World! Message #0"
[INFO] [1769095848.026367609] [talker]: Publishing: "Hello World! Message #1"
[INFO] [1769095849.026754791] [talker]: Publishing: "Hello World! Message #2"
[INFO] [1769095850.027226656] [talker]: Publishing: "Hello World! Message #3"
[INFO] [1769095851.666062107] [talker]: Publishing: "Hello World! Message #4"
```

**Run listener:**
```bash
ros2 run env_check_pkg listener.py
```

**Output (3–4 lines):**
```
[INFO] [1769095984.568011687] [listener]: Listener node started, waiting for messages...
[INFO] [1769095985.027287008] [listener]: Received: "Hello World! Message #138"
[INFO] [1769095986.026905634] [listener]: Received: "Hello World! Message #139"
[INFO] [1769095987.026743093] [listener]: Received: "Hello World! Message #140"
[INFO] [1769095988.027873637] [listener]: Received: "Hello World! Message #141"
```

**Alternative (using launch file):**
```bash
ros2 launch env_check_pkg env_check.launch.py
```

**Screenshot:**  
<img width="2880" height="1660" alt="image" src="https://github.com/user-attachments/assets/abefda61-05bd-4eea-9c2d-9d93f2ffa813" />


---

## 4. Problems Encountered and How I Solved Them

### Issue 1: Missing python3-venv package

**Error message:**
```
The virtual environment was not created successfully because ensurepip is not
available. On Debian/Ubuntu systems, you need to install the python3-venv
package using the following command.

    apt install python3.10-venv
```

**Cause / diagnosis:**  
Ubuntu systems do not include the `python3-venv` package by default. This is a required component for Python virtual environment management tools. WSL2's minimal Ubuntu installation often lacks this package.

**Fix:**

```bash
sudo apt update
sudo apt install python3.10-venv -y

# Then recreate the virtual environment
python3 -m venv .venv
source .venv/bin/activate
```

**Reference:**  
Python official documentation: https://docs.python.org/3/library/venv.html  
Ubuntu package management documentation

---

### Issue 2: ROS 2 version conflict (Iron vs Humble)

**Error message:**
```
ROS_DISTRO was set to 'iron' before. Please make sure that the environment 
does not mix paths from different distributions.
```

**Cause / diagnosis:**  
The system had ROS 2 Iron previously installed, and the `.bashrc` file contained a line sourcing the Iron setup script. When trying to use ROS 2 Humble (required for AAE5303), both versions were being sourced in the same environment, causing environment variable conflicts. This happens when users upgrade or switch between ROS distributions without properly cleaning up the old configuration.

**Fix:**

First, identify which ROS versions are being sourced:

```bash
# Check .bashrc for ROS source lines
grep -n "ros.*setup.bash" ~/.bashrc
```

This revealed:
```
126:source /opt/ros/iron/setup.bash
129:source /opt/ros/humble/setup.bash
```

Solution - remove the old Iron line and keep only Humble:

```bash
# Create backup first (always a good practice)
cp ~/.bashrc ~/.bashrc.backup

# Remove the Iron source line
sed -i '/\/opt\/ros\/iron\/setup.bash/d' ~/.bashrc

# Verify only Humble remains
grep "ros.*setup.bash" ~/.bashrc

# For current terminal, manually reload Humble environment
unset ROS_DISTRO
source /opt/ros/humble/setup.bash

# Verify correct version
echo $ROS_DISTRO  # Should output: humble
```

**Reference:**  
ROS 2 installation documentation: https://docs.ros.org/en/humble/Installation.html  
ROS REP 2000 (ROS 2 Releases and Target Platforms): https://www.ros.org/reps/rep-2000.html

---

### Issue 3: Slow Python package installation

**Phenomenon:**  
Using `pip install open3d` results in very slow installation process, sometimes even timing out.

**Cause / diagnosis:**  
Open3D is a large package (~100MB) containing pre-compiled C++ libraries. Downloading from the default PyPI source can be slow, especially in China.

**Fix:**

Use a domestic mirror source to accelerate downloads:

```bash
# Use Tsinghua mirror source
pip3 install --user -i https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt

# Or permanently configure pip to use mirror source
pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```

**Other available mirror sources:**
- Aliyun: https://mirrors.aliyun.com/pypi/simple/
- USTC: https://pypi.mirrors.ustc.edu.cn/simple/
- Douban: http://pypi.douban.com/simple/

**Reference:**  
Tsinghua University Open Source Software Mirror: https://mirrors.tuna.tsinghua.edu.cn/help/pypi/

---

## 5. Use of Generative AI (Required)

hoose one of the issues above and document how you used AI to solve it.

### 5.1 Exact prompt you asked

**Your prompt:**
```
I'm trying to create a Python virtual environment on Ubuntu 22.04 WSL2, and when I run `python3 -m venv .venv`, I get this error: "The virtual environment was not created successfully because ensurepip is not available." How should I solve this problem?
```

### 5.2 Key helpful part of the AI's answer

**AI's response (relevant part only):**
```
This problem occurs because the Ubuntu system is missing the python3-venv package. 
You need to install it:

sudo apt update
sudo apt install python3.10-venv

After installation is complete, rerun the venv command.

Additionally, if you don't want to use a virtual environment, you can also use 
pip3 install --user to install packages to the user directory without needing 
a virtual environment.
```

### 5.3 What you changed or ignored and why

**Your explanation:**  
- The AI's suggestions were overall correct and safe
- I adopted the second suggestion (using the `--user` flag) because this avoids needing sudo permissions to create a virtual environment
- In production environments, using virtual environments is best practice, but for course exercises, user-level installation is also acceptable
- I verified with the official Python documentation that both methods are valid

### 5.4 Final solution you applied

```bash
# Solution 1: Install venv package and create virtual environment (recommended)
sudo apt install python3.10-venv
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# Solution 2: Use user-level installation (no sudo required)
pip3 install --user -r requirements.txt
```

**Why this worked:**  
Solution 1 installs the missing system package, allowing Python to create isolated virtual environments. Solution 2 bypasses the virtual environment and installs directly to the user's local package directory (`~/.local/lib/python3.10/site-packages/`), which doesn't require system permissions.

---

## 6. Reflection (3–5 sentences)

**Your reflection:**

Through this environment setup, I learned that configuring a robotics development environment on Linux requires careful handling of dependencies. What surprised me most was the subtle differences between WSL2 and native Linux environments, particularly in graphics interface support. Next time, I will first check if all system dependencies are complete and pre-configure package manager mirror sources to speed up installation. Using virtual environments is a good practice to avoid dependency conflicts between different projects. By solving these problems, I now feel more confident about debugging ROS 2 and Python environment issues.

---

## 7. Declaration

✅ **I confirm that I performed this setup myself and all screenshots/logs reflect my own environment.**

**Name:**  
YAN SHENTAO

**Student ID:**  
25132971G

**Date:**  
22/1/2025

---

## Submission Checklist

Before submitting, ensure you have:

- [x] Filled in all system information
- [x] Included actual terminal outputs (not just screenshots)
- [x] Provided at least 2 screenshots (Python tests + ROS talker/listener)
- [x] Documented 2–3 real problems with solutions
- [x] Completed the AI usage section with exact prompts
- [x] Written a thoughtful reflection (3–5 sentences)
- [x] Signed the declaration

---

**End of Report**
