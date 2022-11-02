# stingray_web_interface

# Setup and run

## Install python requirements
```bash
pip3 install requirements.txt
```

## Build
```bash
$ROS_DISTRO=galactic
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
```

## Run

### In first terminal
```bash
source install/setup.bash
ros2 run py_pubsub talker
```

### In second terminal
```bash
source install/setup.bash
uvicorn py_pubsub.listener:app --reload
```

### Then go to http://127.0.0.1:8000