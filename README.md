# sai2-primitives
This is a control library that provides an implementation of basic tasks and primitives for a robot.
It uses the Operatinal Space framework.
It offers the possibility to be used with the [Reflexxes library](http://www.reflexxes.ws/) for trajectory generation (Type II and IV).

Each task is used in a similar manner. At creation, it requires a robot model and an operational space point (and sensors locations if applicable)
There is a function called updateTaskModel that needs to be called after the robot model has been updated in order to compute the task inertia and nullspace projections.
There is a function called computeTorques that computes the robot joint torques that will contribute to this task (needs to be called each control loop)
There are member variable inputs like the gains and the desired position/orientation/force.

## Dependencies
sai2-primitives depends on sai2-model and Eigen3 (> 3.4).

Install eigen 3.4 from source:
- Clone from <https://gitlab.com/libeigen/eigen/-/releases/3.4.0>
- mkdir build && cd build && cmake .. && make && sudo make install 
- Add symlinks to swap from eigen3/Eigen/ to Eigen/:
	- cd /usr/local/include
	- sudo ln -sf eigen3/Eigen Eigen
	- sudo ln -sf eigen3/unsupported unsupported

The examples depend on additional libraries : sai2-simulation, sai2-graphics, sai2-common.

## Build instructions
```
mkdir build
cd build
cmake .. && make -j4
```

## Using Reflexxes Library
If you want to use the Reflexxes Library, you need to get it from [Here](http://www.reflexxes.ws/) (Type II is free) and building it following their instructions.
Then, in the CMakelists.txt file you will see an option "USE_OTG" that you need to put to ON, and you need to specify you path to "OTG_SOURCE_DIR" and "OTG_LIBRARY" in the CMakelists.txt file.
Then, in the tasks, if you set the "_use_interpolation_flag"_ to true, the task will automatically interpolate between the robot current position and desired position with the specified limits on velocity, acceleration and jerk.

## Run the examples
Remember that you need sai2-simulation, sai2-graphics and sai2-common in order to compile and run the examples.
Go to build/examples/desired_example and run the example. For example 2 :
```
cd build/examples/01-puma_control_posori_tasks
./01-puma_control_posori_tasks
```
