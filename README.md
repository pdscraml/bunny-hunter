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

# Current Progress

##### State Machine (PUTTING IT ALL TOGETHER)
- [x] Joystick control
- [x] Master State Machine (calls upper states)
- [ ] Build entire skeleton framework of state machine, then work through each individual module
- [ ] Convert Compeleted Modules to Compatible State Machine “Form”

#### Exploration
- [x] Basic Exploration
- [ ] Fine-tune Random/Intelligent Spins
- [ ] Exploration Jerkiness, from map?
  - Fine-tune gmapping? (i.e. refresh rate)

##### Map-server
- [ ] Can be run on Jackal?
- [ ] Launch on Jackal.launch

#### Check Map Completeness
- [ ] Define threshold and Map "Completeness"
- [ ] Processor Intensive?  Reduce bottlenecks/complexity

##### Easter Basket Hunting
- [x] Basic Ar Tracker
- [x] Waypoint on marker
- [ ] Needs offset from marker
- [ ] Filter orientation to 2D
- [x] Egg Detection

#### Navigation
- [x] Navigation Stack

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
