# [GNC Functions (Python)](../src/iq_gnc/py_gnc_functions.py)

![iq](./imgs/iq.JPG)

The python version of the API has been designed to work as seamlessly and easily as the c++ version of the API. But since the python version has been implemented with classes the user should create an object for the API. 

The `init_publisher_subscriber()` has been removed in the python version and will be initialized when the object of the class is created. 

***Hey there is a [C++ version](../include/gnc_functions.hpp) of the API whose documentation is [here.](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/GNC_functions_documentation.md)***

 ## Example Programs 

 How to run example programs:
 ```bash
$ rosrun iq_gnc <example_program.py>
 ```

> <span style="color:red">Note </span>\
> You have to make the python files executable. You can do this by:

```console
$ cd ~/catkin_ws/src/iq_gnc/scripts/
$ chmod +x *.py 
```

* ### [`square.py`](../scripts/square.py)
  * Example program that will make the drone move in a square.
* ### [`subscriber.py`](../scripts/subscriber.py)
  * Example program showing how to use a ROS subscriber to take input into your drone's guidance node.
* ### [`snr.py`](../scripts/snr.py)
  * Simple search and rescue program that will fly a search pattern until YOLO detects a person. This will trigger the drone to land to deliver the rescue supplies.
* ### [`obs_avoid.py`](../scripts/obs_avoid.py)
  * Example obstacle avoidance program utilizing the potential field method to avoid obstacles.

---

## API Explanation 

How to call/use the API:

```python
	from iq_gnc.py_gnc_functions import *

	drone = gnc_api()
```

### get_current_heading

```python
 | get_current_heading()
```

Returns the current heading of the drone.

**Returns**:

- `Heading` _Float_ - θ in is degrees.

---

### get_current_location

```python
 | get_current_location()
```

Returns the current position of the drone.

**Returns**:

- `Position` *geometry_msgs.msgs.Point()* - Returns position of type Point().

---
### land

```python
 | land()
```

The function changes the mode of the drone to LAND.

**Returns**:

- `0` _int_ - LAND successful.
- `-1` _int_ - LAND unsuccessful.

---

### wait4connect

```python
 | wait4connect()
```

Wait for connect is a function that will hold the program until communication with the FCU is established.

**Returns**:

- `0` _int_ - Connected to FCU.
- `-1` _int_ - Failed to connect to FCU.

---

### wait4start

```python
 | wait4start()
```

This function will hold the program until the user signals the FCU to mode enter GUIDED mode. This is typically done from a switch on the safety pilot's remote or from the Ground Control Station.

**Returns**:

- `0` _int_ - Mission started successfully.
- `-1` _int_ - Failed to start mission.

---

### set_mode

```python
 | set_mode(mode)
```

This function changes the mode of the drone to a user specified mode. This takes the mode as a string. \
`Ex. set_mode("GUIDED")`.

**Arguments**:

- `mode` _String_ - Can be set to modes given in https://ardupilot.org/copter/docs/flight-modes.html
  

**Returns**:

- `0` _int_ - Mode Set successful.
- `-1` _int_ - Mode Set unsuccessful.

---

### set_speed

```python
 | set_speed(speed_mps)
```

This function is used to change the speed of the vehicle in guided mode. It takes the speed in meters per second as a float as the input.

**Arguments**:

- `speed_mps` _Float_ - Speed in m/s.
  

**Returns**:

- `0` _int_ - Speed set successful.
- `-1` _int_ - Speed set unsuccessful.

---

### set_heading

```python
 | set_heading(heading)
```

This function is used to specify the drone's heading in the local reference frame. Psi is a counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

**Arguments**:

- `heading` _Float_ - θ(degree) Heading angle of the drone.

---

### set_destination

```python
 | set_destination(x, y, z, psi)
```

This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

**Arguments**:

- `x` _Float_ - x(m) Distance with respect to your local frame.
- `y` _Float_ - y(m) Distance with respect to your local frame.
- `z` _Float_ - z(m) Distance with respect to your local frame.
- `psi` _Float_ - θ(degree) Heading angle of the drone.

---

### arm

```python
 | arm()
```

Arms the drone for takeoff.

**Returns**:

- `0` _int_ - Arming successful.
- `-1` _int_ - Arming unsuccessful.

---

### takeoff

```python
 | takeoff(takeoff_alt)
```

The takeoff function will arm the drone and put the drone in a hover above the initial position.

**Arguments**:

- `takeoff_alt` _Float_ - The altitude at which the drone should hover.
  

**Returns**:

- `0` _int_ - Takeoff successful.
- `-1` _int_ - Takeoff unsuccessful.

---

### initialize_local_frame

```python
 | initialize_local_frame()
```

This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to.

---

### check_waypoint_reached

```python
 | check_waypoint_reached(pos_tol=0.3, head_tol=0.01)
```

This function checks if the waypoint is reached within given tolerance and returns an int of 1 or 0. This function can be used to check when to request the next waypoint in the mission.

**Arguments**:

- `pos_tol` _float, optional_ - Position tolerance under which the drone must be with respect to its position in space. Defaults to 0.3.
- `head_tol` _float, optional_ - Heading or angle tolerance under which the drone must be with respect to its orientation in space. Defaults to 0.01.
  

**Returns**:

- `1` _int_ - Waypoint reached successfully.
- `0` _int_ - Failed to reach Waypoint.

