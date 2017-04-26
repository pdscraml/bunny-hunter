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
```

## Launching Easter Egg Hunt

Once the Jackal is powered up and online, ssh into it with the following commands.

```
$ ssh rsestudent@jackal2
$ cd bunny-hunter/catkin_ws
$ git pull
$ source devel/setup.bash
$ source remote-jackal.sh
$ roslaunch easter_egg_hunt exploration.launch
```

#### State Machine Operation
- This will launch SICK LMS200 wrapper, USB camera package, Alvar package, gmapping, and move_base.
- Alvar Discovery node is also launched.  This manages the discovery and storing of Ar markers.

Press and hold X on joystick to begin hunt.

- *Start_Pause*: Jackal starts in Start_Pause state
- *BUTTON_PRESSED*: Jackal transitions to Origin_Detect upon button press.
- *Origin_Detect*: Jackal saves origin waypoint
- *ORIGIN-DETECTED*: Once origin waypoint detected Jackal switches to EXPLORATION state.
- *EXPLORATION*: Jackal will explore area, building map.
- *EXPLORATION_COMPLETE*: After map is determined to be complete or timer issued, Jackal transitions to START_GOAL.
- *START_GOAL*: Jackal sets waypoint goal to origin.

Jackal waits for user joystick input when Ar code is ready to be read.

- *GOAL_REACHED*: Once Jackal determines waypoint goal successful and Ar code detected, it transitions to EGG_DETECT state.
- *EGG_DETECT*: Jackal sets waypoint to detected AR at origin. Proceeds to origin to detect eggs.  Jackal will then proceed back to origin.


## View Hunt On Local Machine

```
$ rosrun rviz rviz.rviz
```


# Current Progress

##### State Machine (PUTTING IT ALL TOGETHER)
- [x] Joystick control
- [x] Master State Machine (calls upper states)
- [x] Build entire skeleton framework of state machine, then work through each individual module
- [x] Convert Compeleted Modules to Compatible State Machine “Form”

#### Exploration
- [x] Basic Exploration
- [x] Fine-tune Random/Intelligent Spins

##### Map-server
- [ ] Needed?

#### Check Map Completeness
- [ ] Define threshold and Map "Completeness"
- [ ] Processor Intensive?  Reduce bottlenecks/complexity

##### Easter Basket Hunting
- [x] Basic Ar Tracker
- [x] Waypoint on marker
- [x] Needs offset from marker
- [x] Egg Detection
- [ ] Enable/Disable Discovery

#### Navigation
- [x] Navigation Stack
- [ ] Floating Jackal problem

#### Action Library
- [ ] Wait for goal complete, define state completeness for state transition
- See its usage in the [congenial-couscous](https://github.com/RogerGomes29/congenial-couscous) repo.

## Final Testing Checklist
- [ ] Optimize time
- [ ] Explore without jerkiness or collisions
- [ ] Able to complete full map
- [ ] State Machine performance (Changing state when desired)
- [ ] Able to discover and save WP
- [ ] Able to return to WP
- [ ] Able to count eggs at WP

# Calibration

The PlayStation3 Eye camera was calibrated using [this tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).
