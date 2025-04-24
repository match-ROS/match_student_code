# Dynamic Model for Mobile Robots
## Overview

The ROS package *dynamic_model* implements a dynamic model node for mobile robot applications. It is designed to work in both simulation and real-world scenarios by fusing sensor data from other nodes to model the robot's dynamics. This package is intended for developers, students, and institute employees working with mobile robots. The source folder additionally provides a test environment to test the operation of the dynamic model with an agilex scout mini robot.

**Author:** Oliver Maas

**E-Mail:** o.maas@stud.uni-hannover.de

The node *dynamic_model*:
- **Subscribes to:**
  - **IMU Data:** For orientation and linear acceleration.
  - **Encoder Data:** For wheel odometry.
- **Publishes:**
  - `/robot_state`: Full state vector including dynamics.
  - `/dynModForce`: Forces computed from the dynamic model.
  - `/dynModMoment`: Moments (torques) computed from the dynamic model.
- **Uses:** 
  - A custom ROS message (`StateVector.msg`) defined in the `custom_msgs` package (a stamped float array).
  - Sensor data interpolation, kinematic models (*odometrie_mecanum.py*, *odometrie_tracked.py*, *odometrie_differential.py*), and recursive least squares (*rls.py*) for online parameter identification.
- **Tested with:** ROS Noetic, Python 3.8.10.

Additional details regarding the mathematics and algorithms can be found in the associated master thesis.

## Installation

1. **Prerequisites:**
   - ROS Noetic
   - Python 3.8.10
   - Required packages (as listed at the top of the program file)
   - A custom ROS message package named `custom_msgs` (with a message file `StateVector.msg`)

2. **Building the Package:**

   Navigate to your catkin workspace and run:

   ```bash
   catkin_make
   ```

3. **Sourcing the Workspace:**

   Make sure to source your workspace:

   ```bash
   source devel/setup.bash
   ```

## Configuration

The robot is configured using the `robot_settings.json` file. This file contains parameters such as:
- **Robot type**
- **Actuated wheels**
- **Dimensions**
- **Spring-damper values**
- **Wheel positions**
- **Masses**
- **Sensor locations, types, and topics**

### Sample `robot_settings.json`


```json
{
    "robot_name": "ScoutMini",  
    "robot_typ" : "tracked", 
    "robot_typ_comment" : "Choose: Tracked (Kettenantrieb), omnidirectional, differential",
    "motor_count": 4,      
    "dimension":{
        "_comment": "Width, Depth, heigth, track_width & wheel_radius = [m]; spring_stiffness = [N/m]; damping = [Nsm]",
        "width": 0.3,
        "depths": 1.0,
        "heigth": 0.4,
        "track_width": 0.418,
        "wheel_base" : 0.464,
        "wheel_radius": 0.08,
        "spring_stiffness": 25000,
        "damping": 200,
        "wheel_position":{
            "front_left":   [0.2319755, 0.2082515, -0.100998],
            "front_right":  [0.2319755, -0.2082515, -0.100998],
            "rear_left":    [-0.2319755, 0.2082515, -0.100998],
            "rear_right":   [-0.2319755, -0.2082515, -0.100998]
        }
   },
   "mass_mainframe": 60.0, 
   "wheel_mass" : 3.0,    
   "sensors": {
       "imu": {
       "update_rate": 10.0,
       "topic": "/imu_data"
       },
       "encoder":{
            "_comment_encoder": "Encoder Settings für Linke und rechte Seite. Mögliche Einheiten: Winkelgeschwindigkeit rad/s, ",
            "left_encoder":{
                    "update_rate": 10.0,
                    "topic": "/joint_states",
                    "unit": "rad",
                    "sign": "minus"
                },
            "right_encoder":{
                "update_rate": 10.0,
                "topic": "/joint_states",
                "unit": "rad",
                "sign": "plus"
        }
        }
   }
}
```

## Usage

To use the dynamic model it can be started in multiple ways:

- **Directly from the terminal:**

  ```bash
  python src/dynamic_model/src/dynamic_model.py
  ```
  This runs the node in a standalone terminal. It requires Gazebo to run simultaneusly.

- **Via a ROS launch file:**

  You can include the dynamic model as a node in a custom launch file. Example launch file: 

  ```xml
    <launch>
      <!-- [Your Code] -->

      <!-- Start the dynamic model -->
      <node pkg="dynamic_model"
            type="dynamic_model.py"
            name="dynamic_model"
            output="screen" />
    </launch>
  ```

### Topics

The node subscribes to the following topics:
- **IMU Topic:** (e.g., `/imu_data`)
- **Encoder Topics:** (e.g., `/joint_states`)

It publishes data to:
- `/robot_state`
- `/dynModForce`
- `/dynModMoment`

## Additional Scripts

1. **Monitoring:**
    There is a script, `record_topics.py`, which:
    - Captures the simulation environment’s ground truth values.
    - Captures the dynamic model's outputs.
    - Saves the data into different CSV files for further analysis.
    It can be used to test the dynamic model and save the results

2. **agilex_scout**
    This package includes the Agliex Scout Mini with spring damper simulation.
    It can be used to simulate the dynamic model in Gazebo. 

3. **test_dynamic_model**
    This package inclues a test environment with two worlds in wich the Agilex Scout Mini is beeing spawned. The dynamic model can be tested in these worlds.


## Testing & Troubleshooting

- **Example Testing:**
  Use the provided test package *test_dynamic_model*:
  1. Add the src folder to a catkin workspace
  2. Navigate to your catkin workspace and run:
   ```bash
   catkin_make
   ```
  
  3. Launch the test world:
  ```bash
   roslaunch test_dynamic_model plainworld.launch
  ```
  4. Make sure to source your workspace:
   ```bash
   source devel/setup.bash
   ```

  5. In a different termianl start the dynamic_model:
  ```bash
    rosrun dynamic_model dynamic_model.py
  ```

  6. Start some sort of controller to navigate the robot e.g.:
  ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```

  7. In a different Termial run the *record_topics.py* script to save the topics data for later visualisation:
  ```bash
    rosrun dynamic_model record_topics.py
  ```

- **Troubleshooting:**
  - Ensure the custom message package `custom_msgs` is built and sourced.
  - Check that the correct topics are being published/subscribed.
  - Refer to the ROS console for any error messages.

## Additional Documentation

The mathematical derivations, algorithms, and detailed explanations of the dynamic model and estimation techniques are described in the associated master thesis "Entwicklung eines dynamischen Modells zur flexiblen Implementierung auf mobilen Robotersystemen" / Development of a dynamic model for flexible implementation on mobile robot systems".

