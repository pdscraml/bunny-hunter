# Final Project: Jackals hunt rabbits right?

- Map a room or other enclosed area
- Explore the map looking for bunnies, each will have an associated Alvar marker.
- Set the bunny location as waypoints in the map.
- Return to the starting point and pose.
- At the starting point you will find an Alvar marker matching one of the bunnies.
- Proceed to that waypoint.
- Count the number and color of each Easter Egg in the bunny's basket. The basket and eggs will be situated similar to the image above.
- Return to the starting point.
- Provide some sort of visual display, pop-up, with the number of eggs and location (i.e. Alvar marker #).
- Final presentations on April 19.

## Guidelines
- Interact via the controller. (Start/Stop)
- Time is a factor
- Avoid collisions
- Smooth motion
- Three trials
- Document approach on Git
- Trial run week before
- Team leader will give a presentation covering approach. Following the competition.
- Points given for style and creativity.
- Sequential behavior should be enabled using 'smach'.

## Team
- [Ian Wakely](https://github.com/raveious)
- [Akhil Kurup](https://github.com/amkurup)
- [Phillip Scramlin](https://github.com/pdscraml) (Project Leader)

## Local Development

After cloning this repository, follow these steps to setup the development environment
```
$ cd catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ source devel/setup.bash
$ source remote-jackal.sh
$ source remote-jackal.sh jackal2
```

## Jackal Development

Once the Jackal is powered up and online, ssh into it with the following commands.

```
$ ssh rsestudent@jackal2
$ cd bunny-hunter/catkin_ws
$ git pull
$ source devel/setup.bash
$ source remote-jackal.sh
$ roslaunch easter_egg_hunt jackal.launch
```

## Launching Easter Egg Hunt

For launching the easter egg hunt run the following commands on the Jackal.

```
$ roslaunch easter_egg_hunt jackal.launch
```

Then run the following commands in the local machine.
```
$ roslaunch easter_egg_hunt exploration.launch
```

#### State Machine Operation
- This will launch SICK LMS200 wrapper, USB camera package, Alvar package, gmapping, and move_base.
- Alvar Discovery node is also launched.  This manages the discovery and storing of Ar markers.

Press and hold X on joystick to begin hunt.

- **Start_Pause**: Jackal starts in Start_Pause state
- *Button_Pressed*: Jackal transitions to Origin_Detect upon button press.
- **Enable_Discovery**: Alvar marker discovery is enabled
- *Waypoints_Enabled*: Jackal transitions to Origin_Detect after Ar discovery enabled.
- **Origin_Detect**: Jackal saves origin waypoint
- *Origin_Detected*: Once origin waypoint saved Jackal switches to EXPLORATION state.
- **Exploration**: Jackal will explore area, building map and detecting Alvar Marker.
- *Exploration_Complete*: After map is determined to be complete or timer issued, Jackal transitions to START_GOAL.
- **Start_Goal**: Jackal sets waypoint goal to origin.
- *GOAL_REACHED*:  Once Jackal determines waypoint goal successful it transitions to Disable_Discovery
- **Map_Saver**: Jackal saves map.
- *MAP_COMPLETE*: Jackal transitions after map is saved.
- **Disable_Discovery**: Ar Marker discovery disabled.
- *Waypoints_Disabled*:  Transitions to Marker_Display_Wait after Ar Marker discovery disabled.
- **Marker_Display_Wait**: Jackal waits for button press to detect marker.

Press joystick x button to enable Ar detection.

- *Button_Pressed*: After button press, Jackal transitions to Select_Bunny state.
- **Select_Bunny**: Jackal waits for service message that Ar Marker was detected.
- *Waypoint_Selected*: Transition to Bunny_Nav State
- **Bunny_Nav**: Set move_base action goal to designated bunny.
- *Button_Pressed*: After waypoint successfully reached, Jackal transitions to Egg_Detect state.
- **Egg_Detect**: Jackal runs egg detection.

## View Hunt On Local Machine

```
$ roslaunch easter_egg_hunt rviz.launch
```

# Calibration

The PlayStation3 Eye camera was calibrated using [this tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).

# Results

https://www.youtube.com/playlist?list=PLiyhIGkouc_11FCaG2WNUd6RYeX24-bG-
