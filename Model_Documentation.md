# Project Writeup

Several concepts from the lessons were applied to achieve path planning. A FSM was used to differentiate the car's behavior and trajectory based on cost functions. Jerk minimizing trajectories were used to keep jerk and accleration below project requirements during the initial movement. A spline is used to calculate a smooth polynomial for trajectories.

## Splines

To generate a trajectory, the basic principles from the project Q&A were used, such as using frenet coordinates and using splines to append new trajectory points. A spline is created using the furthest two points of the previous trajectory and some projected points (in Frenet, but transformed to XY) of where we want to car to drive towards. The lane that the car gravitates toward can be controlled by the Frenet points. The distance between each spline point is determined by the desired velocity of car. XY points calculated from transformed points off the spline polynomial can be appened to the previous trajectory. 

## State Machine

Three states were implemented: Ready, Keep Lane, and Lane Change. Since. Because of the use of splines, lane changes are as simple as changing the current lane variable, which also inherently makes no differentiation between left and right lane changes.

#### Ready State

When the car transitions from zero to higher velocities, a trajectory must be carefully calculated to ensure jerk and acceleration doesn't exceed the project requirements of 10 m/s<sup>3</sup>  and 10 m/s<sup>2</sup> respectively. To achieve this, JMT is used in conjuction with splines . JMT is calculated with a zero state and final state of 50m, 20 m/s, and 1 m/s<sup>2</sup> in 5 seconds.  These parameters were chosen with an octave script. 

![ ](/home/peter/CarND/Term3/CarND-Path-Planning-Project/JMT.png  "JMT Calculations")

#### Keep Lane State

Once the Vehicle has reached the desired velocity, the car enters the Keep Lane state. From there a desired velocity is maintained and incremented or decremented as needed. If the sensor fusion detects a vehicle a certain distance ahead moving slower than the maximum allowed velocity, it will decrement it's reference velocity until a minimum buffer distance is achieved between the lead and ego car. If the reference velocity is lower than the maximum allowed velocity, the car will calculate cost functions to determine a safe lane to merge into.

#### Lane Change State

When the car moves into a lane change state, the same spline/trajectory generation method is used, but with different target points for the spline. With the Keep Lane, the spline targets a point 30m ahead, but for the lane change, it targets a point 50 m ahead in the adjacent lane. This allows for a smoother curve with less jerk during the lange change.

##Sensor Fusion Data

Sensor fusion data contained information on all cars within a certain distance from the ego car, but only the cars directly in front and behind were relevant when attempting to understand the local environment. The sensor fusion data was parsed into a data structure that helped determine distance/velocity of cars directly trailing and leading the ego car for each lane. This structure was helpful for determining lane velocities and cost function calculations. 

## Cost Functions

Cost functions were implemented to determine the available space for the ego car to merge into adjacent lanes, where a cost of 1 indicates an impossible move and a cost near zero indicates a safe move. Using the velocity and distance of trailing and leading cars, an estimate of where that particular car will be located during the time it takes for the ego car to merge can be calculated. Using this estimate and buffer zones, a cost can be calculated. If the lead/trail car's estimated location inside the buffer zone, the cost will equal 1. Outside of the buffer zone, the cost will have an exponential decay. The leading car 

![ ](/home/peter/CarND/Term3/CarND-Path-Planning-Project/trail_cost_function.png  "Trail Cost Function")

![ ](/home/peter/CarND/Term3/CarND-Path-Planning-Project/lead_cost_function.png  "Lead Cost Function")