# Mobile-Manipulation
## Project Description: 
The program plans a trajectory for the end-effector of the youBot (a mobile base with four mecanum wheels and a 5R robot arm) and performs feedforward plus feedback control to drive the youBot to pick up a block at a specified location, carry it to a desired location, and put it down. (Please see Mobile Manipulation Capstone website for more detail)

## Implementation:
My program contains three important modules:
```
1.TrajectoryGenerator: generate the reference trajectory for the end-effector frame
2.NextState: compute the new config of youBot after a timestep with the given speed of arm joints and wheel.
3.FeedbackControl: generate control signals (twist) that drive youBot to approach the desired trajectory.  
  Feedforward and feedback PI control are integrated into this controller.
```

The main procedures of this program is:
```
1.Generate reference end-effector trajectory using youBot initial reference config and cube initial/goal 
  configs specified by the user Loop through each pose in the trajectory. 
2.In a single loop:
  a.Feed the current actual pose, current reference pose and the next reference pose into the controller 
    to obtain the control signal (twist)
  b.Convert the twist into angular speed of arm joints and wheels
  c.Simulate the next state of youBot using the angular speed
3.Write configurations of youBot and error twists from the simulation loop into csv files
```

## Results:
As a result, my program can make the youBot complete the task starting from a random(with a proper deviation) initial configuration, as long as the controller is well-tuned. 

## Enhancement: 
I performed singularity avoidance by adding tolerance when computing pseudo-inverse of manipulator Jacobian. And for testing convenience, I wrote a script that generate a random actual config of youBot that deviates properly from the given reference config

## Additional information that might help:
```
1.If you are using PI control and find that the error twist displays a non-converging, chaotic pattern, 
  it might be the internal computational error from matlab. If adjusting the gains does not solve 
  the problem, uncomment the disp in FeedbackControl.m and try again.
2.If you want to try a different initial reference end-effector pose(the start of the reference trajectory),
  open Scene3_youBot in your CoppeliaSim (V-rep), play with the joints slider to get as close to your 
  desired end-effector pose as possible and use inverse kinematic (IKinBody in Modern Robotics package) to 
  solve for exact joint angles.
