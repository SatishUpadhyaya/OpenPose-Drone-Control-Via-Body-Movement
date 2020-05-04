# Robotics-Final-Project
Final project for Introduction to Robotics

## Commands to Launch
```sh
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
gazebo --verbose worlds/iris_arducopter_runway.world
roslaunch mavros apm.launch
```

## Commands to Run
1. `chmod +x init.py`
2. `python3 init.py`

### Helpful Tip(s):
1. Change line number 5 in `apm.launch` file to: `<arg name="fcu_url" default="udp://127.0.0.1:14551@" />`

### Helpful Commands:
1. `rosservice call /mavros/set_mode 0 guided`
2. `rosservice call /mavros/cmd/arming true`
3. `rosservice call /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 10.0}"`
4. `rosservice call /mavros/set_mode 0 land`
5. `rosservice call /mavros/cmd/arming false`
