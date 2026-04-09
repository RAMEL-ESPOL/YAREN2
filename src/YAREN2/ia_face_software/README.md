# Multi-Platform AI-Based Software for the Accompaniment and Support of Children in Hospitals

![Image of Yaren](https://github.com/RAMEL-ESPOL/SoftwareInteractive/blob/main/README_FILES/MultiplatformSoftware.png)

## Execution Instructions

Clone the repository

```bash
git clone https://github.com/RAMEL-ESPOL/SoftwareInteractive.git  
```

Navigate to the following directory

```bash
cd SoftwareInteractive  
```

Install the following dependencies

```bash
sudo apt install -y portaudio19-dev python3-pyaudio wmctrl ffmpeg  
```

```bash
pip install -r requirements.txt  
```

If ROS is not installed, visit: [https://wiki.ros.org/ROS/Installation](https://wiki.ros.org/ROS/Installation)
It is recommended that the Python and Python Debugger extensions be installed.

Remember to grant execution permissions to the files using `chmod +x filename.launch`

```bash
catkin_make  
```

```bash
source devel/setup.bash  
```

To run the facial expression system

```bash
roslaunch software_interactive multiplatform.launch  
```

## Body Movements and Games

![Image of Yaren](https://github.com/RAMEL-ESPOL/SoftwareInteractive/blob/main/README_FILES/22d.png)

If you want to access the repository related to motor control to enable non-verbal interactions (dances, body movements, etc.) and play on the LCD screen through the robot's joints, visit the following link: [https://github.com/RAMEL-ESPOL/YAREN.git](https://github.com/RAMEL-ESPOL/YAREN.git)

