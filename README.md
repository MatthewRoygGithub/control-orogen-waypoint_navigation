# Waypoint Navigation

## A pure-pursuit based path following algorithm for planetary rover models

This component and library provide path following functionality. The inputs of the main component (Task.cpp in orogen) are the current pose of the robot and the path to be followed, represented as `std::vector` of Waypoints. Using the current pose and the path, the component outputs a `MotionCommand2D` (translational and rotational velocity, representing the turn arc) which is then passed into the Locomotion Control. The component in the driving mode acts as a P controller on the current pose only. For visualization, the current lookahead point is also an output. All the path following functionaloity si handled in the C++ library and is framework-independent

**Author: Jan Filip
Contact: Martin Azkarate  
Affiliation: Automation and Robotics Laboratories, ESTEC, ESA**

## Algorithm description
For the description of the path following algorithm, refer to `README.md` in 
[control-waypoint_navigation](https://github.com/exoter-rover/control-waypoint_navigation "Waypoint Navigation Library").

## Usage

#### Inputs and Outputs
For all the inputs and outputs of the component, refer to `waypoint_navigation.orogen`. The necessary inputs are:
* **`trajectory`** (/std/vector</base/Waypoint>)
Trajectory the robot should follow
* **`pose`** (base/samples/RigidBodyState)
Position and orientation of the vehicle
The resulting output is
* **`motion_command`** (base/MotionCommand2D)
Commanded turn arc represented as `tv` translational and `rv` rotational velocity `r_{turn} = \frac{tv}{rv}`

#### Config Parameters
The path following component is configurable using following parameters:
* **`ptConfig:`**
    * **`minTurnRadius`** (/double)
    Minimum turn radius of the vehicle (in meters).
    * **`translationalVelocity`** (/double)
    Target translational velocity (m/s).
    * **`rotationalVelocity`** (/double)
    Target point-turn rotational velocity (rad/s).
    * **`corridor`** (/double)
    max. lateral deviation from the nominal path (which is given as straight line connection of the consecutive waypoints (in meters).
    * **`lookaheadDistance`** (/double)
    The default lookahead distance `d_{lh}` (in meters).
    * **`backwards`** (/bool)
    Allows backward motion.

* **`tolHeading`** (/bool)
Default goal heading tolerance (deg)
* **`tolPos`** (/bool)
Default distance from the goal position tolerance (m)

Alignment PD controller is configured using
* **`pdConfig:`**
    * **`P`** (/double)
    P gain of the controller
    * **`D`** (/double)
    D gain of the controller
    * **`saturation`** (/double)
    Heading error saturation (rad)
