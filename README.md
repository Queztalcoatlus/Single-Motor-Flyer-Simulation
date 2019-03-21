# Single Motor Flyer Simulation
The project aims to create a simulation model of a single-motor powered drone based on “A controllable flying vehicle with a single moving part” (W. Zhang, M. Mueller, and R. D’Andrea, 2016). The dynamics of the drone and `Visualize.m` referenced <https://github.com/gibiansky/experiments/tree/master/quadcopter>.

## Getting Started
Simulink 2018b is required to run `Single_motor_flyer_controller.slx`. All scripts are in MATLAB format.

## Design
The simulation consists of three parts: calculation of the hover solution, linearization around hover and the control loop.
`Hover.m` calculates the hover solution given a set of parameters including:
mass and momentum of inertia of the propeller and the frame, position of the propeller relative to the center of mass of the frame, the drag matrix of the frame, torque and thrust constants of the propeller.
`Single_motor_flyer_controller.slx` contains the control loop. The drone starts at [0;0;0], and the desired position is set in the "Desired Position" block. The "Flyer" block updates the dynamics of the drone every time step. 
`Plot_Data.m` plots the state of the drone during each simulation run.
`Visualize.m` draws the trajectory and attitude of the drone.
`Linearization.m` takes the same parameters from `Hover.m` and the time constant of the motor and produces a gain vector K. The gain K is then plugged into the Attitude Control block in `Single_motor_flyer_controller.slx`.

## Running and Tests
First open `Single_motor_flyer_controller.slx`, and click **Run**.
Then run 
```
Plot_Data.m;
```
to plot figures.
Run
```
Visualize(results);
```
to generate the trajectory.