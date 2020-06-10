# Robotics-Final-Project
Final project for Introduction to Robotics

## Commands to Launch
```sh
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
gazebo --verbose worlds/iris_arducopter_runway.world
roslaunch mavros apm.launch
```

## Commands to Run
**For keyboard controls:**
```sh
chmod +x keyboardControl.py
python3 keyboardControl.py
```
Example (Controlled through the PyGame window):

[![Gazebo Obstacle Course Keyboard Control](https://media.giphy.com/media/hSXptal3OhEWiphQxv/giphy.gif)](https://www.youtube.com/watch?v=rSxEVxJFBDE&t=5s "Gazebo Obstacle Course Keyboard Control")

**For arUco tracking controls:**
```sh
chmod +x arucoControl.py
python3 arucoControl.py
```
Example:

[![Gazebo Obstacle Course ArUco Control](https://media.giphy.com/media/MY7DW7ATVe2sTSinTq/giphy.gif)](https://www.youtube.com/watch?v=8EMqUAViw-A "Gazebo Obstacle Course ArUco Control")

**For OpenPose:**
```sh
Coming Soon
```

### Helpful Tip(s) for MavRos:
1. Change line number 5 in `apm.launch` file to: `<arg name="fcu_url" default="udp://127.0.0.1:14551@" />`
2. Change line number 117 in `apm_config.yaml` with `mav_frame: LOCAL_NED` to `mav_frame: BODY_NED`. This is to make sure that the drone's setpoint velocity commands are relative to its frame.
3. Change `<gui>` settings in `/usr/share/gazebo-9/worlds/iris_arducopter_runway.world` from:
```yaml
<gui>
    <camera name="user_camera">
        <pose>-5 0 1 0 0.2 0</pose>
    </camera>
</gui>
```
to: 
```yaml  
<gui>
    <camera name="user_camera">
        <track_visual>
            <name>iris_demo</name>
            <static>true</static>
            <use_model_frame>true</use_model_frame>
            <xyz>-3 0 0</xyz>
            <inherit_yaw>true</inherit_yaw>
        </track_visual>
    </camera>
</gui>
```
for a third-person view for the drone.

### Helpful Commands for MavRos:
```sh
rosservice call /mavros/set_mode 0 guided
rosservice call /mavros/cmd/arming true
rosservice call /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 10.0}"
rosservice call /mavros/set_mode 0 land
rosservice call /mavros/cmd/arming false
```
Or on the terminal that you enter `sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console`:
```sh
mode guided
arm throttle
takeoff <numValue>
mode land
```
